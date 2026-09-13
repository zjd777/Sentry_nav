#include "serial_driver_ch343/RMserialDriver.hpp"
#define DEBUG_SERIAL_DRIVER 0

#include <algorithm>
#include <stdexcept>

// 引入必要的消息类型
#include "geometry_msgs/msg/twist.hpp"

namespace rm_serial_driver
{
/**
 * @brief 串口驱动节点构造函数 - 精简版，仅处理底盘控制
 * @param options ROS2节点选项
 * 功能：初始化串口驱动节点，创建cmd_vel订阅者，打开串口并启动定时器和接收线程
 */
SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("chassis_serial_driver", options),  // 修改节点名
  // 创建串口配置对象：波特率2000000，数据位8，无校验位，2个停止位
  config(std::make_shared<SerialConfig>(2000000, 8, false, StopBit::TWO, Parity::NONE)),
  port(std::make_shared<Port>(config))
{
  RCLCPP_WARN(get_logger(), "Starting Chassis Serial Driver Node!");

  // 移除所有云台相关的参数和组件
  // 只保留基础的串口功能

  // 创建cmd_vel话题订阅者，用于接收导航速度命令
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SensorDataQoS(),
    std::bind(&SerialDriverNode::cmdVelCallback, this, std::placeholders::_1));

  // 可选：创建调试信息发布者
  debug_pub_ = this->create_publisher<std_msgs::msg::Float64>("/chassis_debug", 10);

  // 打开串口。设备未插入时低频重试，避免启动阶段忙等占满 CPU。
  while (rclcpp::ok() && !port->isPortOpen()) {
    if (port->openPort() < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for CH343 serial device: /dev/ttyCH343USB0/1/2");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  if (!port->isPortOpen()) {
    throw std::runtime_error("Serial driver stopped before the CH343 port was opened");
  }

  // 创建定时器，每1ms执行一次transmit函数发送数据
  timer_transmit = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&SerialDriverNode::transmit, this));
  
  // 创建接收线程，专门处理串口数据接收
  rx_thread = std::thread(&SerialDriverNode::rx, this);
}

SerialDriverNode::~SerialDriverNode()
{
  running_.store(false);
  if (timer_transmit) {
    timer_transmit->cancel();
  }
  if (port) {
    port->closePort();
  }
  if (rx_thread.joinable()) {
    rx_thread.join();
  }
}

/**
 * @brief 数据接收线程函数
 * 功能：持续接收串口数据并进行解码处理
 */
void SerialDriverNode::rx()
{
  while (running_.load() && rclcpp::ok()) {
    receive();
    pkgState = PkgState::COMPLETE;
    while (receive_buffer.size() > 0 && pkgState != PkgState::HEADER_INCOMPLETE &&
           pkgState != PkgState::PAYLOAD_INCOMPLETE) {
      pkgState = decode();
    }
  }
}

/**
 * @brief 数据包解码函数
 * @return PkgState 返回数据包解析状态
 * 功能：从接收缓冲区中解析完整的数据包，包括CRC校验
 */
PkgState SerialDriverNode::decode()
{
  int size = receive_buffer.size();

  for (int i = 0; i < size; i++) {
    if (receive_buffer[i] == 0xAA) {
      if (i + int(sizeof(Header)) > size) {
        return PkgState::HEADER_INCOMPLETE;
      }

      std::copy(
        receive_buffer.begin() + i, receive_buffer.begin() + i + sizeof(Header), decodeBuffer);
      crc_ok_header = crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

      if (!crc_ok_header) {
        RCLCPP_ERROR(get_logger(), "CRC error in header !");
        error_sum_header++;
        try {
          receive_buffer.erase(
            receive_buffer.begin() + i, receive_buffer.begin() + i + sizeof(Header));
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
        }
        return PkgState::CRC_HEADER_ERRROR;
      }

      this->header = arrayToStruct<Header>(decodeBuffer);

      // pkg length = payload(dataLen) + header len (include header crc) + 2crc
      if (i + int(header.dataLen) + int(sizeof(Header)) + 2 > size) {
        return PkgState::PAYLOAD_INCOMPLETE;
      }

      std::copy(
        receive_buffer.begin() + i,
        receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2, decodeBuffer);
      crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer, header.dataLen + sizeof(Header) + 2);

      if (!crc_ok) {
        error_sum_payload++;
        try {
          receive_buffer.erase(
            receive_buffer.begin(),
            receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
        }
        return PkgState::CRC_PKG_ERROR;
      }

      // 完整数据包处理
      try {
        receive_buffer.erase(
          receive_buffer.begin(), receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
      }

      pkg_sum++;

      // 处理接收到的数据包（仅处理底盘相关消息）
      processChassisMessage(decodeBuffer);
      return PkgState::COMPLETE;
    }
  }
  receive_buffer.erase(receive_buffer.begin(), receive_buffer.end());
  return PkgState::OTHER;
}

/**
 * @brief 处理底盘相关消息
 * @param data 指向已解码数据包的指针
 * 功能：处理从STM32接收到的底盘状态反馈消息（修复问题4）
 */
void SerialDriverNode::processChassisMessage(uint8_t * data)
{
  // 根据协议ID分类处理
  Header header = arrayToStruct<Header>(data);
  
  switch (header.protocolID) {
    case CHASSIS_MSG:  // 修复问题1：使用正确的协议ID常量（0xA3）
      {
        // 修复问题4：使用协议定义的结构体类型
        ChassisMsg feedback = arrayToStruct<ChassisMsg>(data);
        RCLCPP_DEBUG(get_logger(), "Chassis feedback received: xVel=%.2f, yVel=%.2f, wVel=%.2f",
                     feedback.xVel, feedback.yVel, feedback.wVel);
        // RCLCPP_INFO(get_logger(), "Chassis feedback received: xVel=%.2f, yVel=%.2f, wVel=%.2f",
        //              feedback.xVel, feedback.yVel, feedback.wVel);
        // 这里可以处理底盘反馈数据，如发布到ROS话题
      }
      break;
    default:
      // 忽略其他协议消息
      RCLCPP_DEBUG(get_logger(), "Ignoring protocol ID: 0x%02X", header.protocolID);
      break;
  }
}

/**
 * @brief 串口数据接收函数 - 修复版本
 * @return int 实际读取的字节数
 * 功能：修复读取错误处理逻辑，避免串口频繁重启
 */
int SerialDriverNode::receive()
{
  int read_num = 0;

  if (!port->isPortOpen() || port->fd < 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return read_num;
  }

  read_num = read(port->fd, receiveBuffer, 64);
  
  if (read_num > 0) {
    // 成功读取到数据
    read_sum += read_num;
    receive_buffer.insert(receive_buffer.end(), receiveBuffer, receiveBuffer + read_num);
  } else if (read_num == 0) {
    // 正常情况：没有数据可读，不是错误！
    // 什么都不做，或者可以添加一个小的延时
  } else {
    // read_num < 0：真正的错误
    if (running_.load()) {
      RCLCPP_ERROR(get_logger(), "Can not read from the serial port!");
      port->closePort();
      port->openPort();
    }
  }
  return read_num;
}

/**
 * @brief cmd_vel话题回调函数
 * @param msg 速度命令消息共享指针
 * 功能：处理接收到的导航速度命令，构造底盘控制数据包并添加到发送缓冲区
 */
void SerialDriverNode::cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    trans_pkg_sum++;
    uint8_t buffer[sizeof(ChassisCommand)];
    ChassisCommand chassis_command_msg;

    // 设置数据包头部信息
    chassis_command_msg.header.dataLen = sizeof(ChassisCommand) - sizeof(Header) - 2;
    //协议ID常量（0xB3）
    chassis_command_msg.header.protocolID = CHASSIS_CMD;

    // 设置底盘控制参数（修复问题2：使用协议定义的正确字段名）
    chassis_command_msg.vel_x = static_cast<float>(msg->linear.x);   // 前进/后退速度 (m/s)
    chassis_command_msg.vel_y = static_cast<float>(msg->linear.y);   // 横向速度 (m/s)
    chassis_command_msg.vel_w = static_cast<float>(msg->angular.z);  // 旋转速度 (rad/s)

    // 可选：限制速度范围
    // chassis_command_msg.vel_x = std::clamp(chassis_command_msg.vel_x, -2.0f, 2.0f);
    // chassis_command_msg.vel_y = std::clamp(chassis_command_msg.vel_y, -2.0f, 2.0f);
    // chassis_command_msg.vel_w = std::clamp(chassis_command_msg.vel_w, -3.14f, 3.14f);

    // 添加CRC校验
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&chassis_command_msg), sizeof(Header));
    crc16::Append_CRC16_Check_Sum(
      reinterpret_cast<uint8_t *>(&chassis_command_msg), sizeof(ChassisCommand));
    
    // 将结构体转换为字节数组
    structToArray(chassis_command_msg, buffer);
    
    //    // ==================== 添加打印开始 ====================
    // // 打印即将发送的ROS命令
    // RCLCPP_INFO(get_logger(), "收到ROS速度命令: vx=%.2f, vy=%.2f, wz=%.2f", 
    //             msg->linear.x, msg->linear.y, msg->angular.z);
    
    // // 打印即将发送的二进制数据包（十六进制格式）
    // std::stringstream hex_stream;
    // hex_stream << "将要发送的二进制数据包（" << sizeof(buffer) << "字节）: ";
    // for (size_t i = 0; i < sizeof(buffer); i++) {
    //     hex_stream << std::hex << std::setw(2) << std::setfill('0') 
    //                << static_cast<int>(buffer[i]) << " ";
    //     if ((i + 1) % 8 == 0) hex_stream << " "; // 每8字节加一个空格
    // }
    // RCLCPP_INFO(get_logger(), "%s", hex_stream.str().c_str());
    // // ==================== 添加打印结束 ====================
    // 加锁保护发送缓冲区
    std::lock_guard<std::mutex> lockf(transmit_mutex);
    
    // 将数据包添加到发送缓冲区
    transmit_buffer.insert(transmit_buffer.end(), buffer, buffer + sizeof(ChassisCommand));

    // 修复问题2：使用正确的字段名输出调试信息
    RCLCPP_DEBUG(get_logger(), "Sent chassis command: vx=%.2f, vy=%.2f, w=%.2f", 
                 chassis_command_msg.vel_x, chassis_command_msg.vel_y, chassis_command_msg.vel_w);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending chassis command: %s", ex.what());
    port->closePort();
    port->openPort();
  }
}

/**
 * @brief 数据发送函数
 * @return int 实际写入的字节数
 * 功能：从发送缓冲区读取数据并通过串口发送
 */
int SerialDriverNode::transmit()
{
  std::vector<uint8_t> buffer;
  {
    std::lock_guard<std::mutex> lock(transmit_mutex);
    if (transmit_buffer.empty() || !running_.load()) {
      return 0;
    }

    const auto bytes_to_write = std::min<std::size_t>(TRANSMIT_BUFFER, transmit_buffer.size());
    buffer.assign(transmit_buffer.begin(), transmit_buffer.begin() + bytes_to_write);
    transmit_buffer.erase(transmit_buffer.begin(), transmit_buffer.begin() + bytes_to_write);
  }

  if (!port->isPortOpen() || port->fd < 0) {
    std::lock_guard<std::mutex> lock(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.begin(), buffer.begin(), buffer.end());
    return 0;
  }

  write_num = write(port->fd, buffer.data(), buffer.size());

  if (write_num < 0) {
    RCLCPP_ERROR(get_logger(), "Can not transmit");
    {
      std::lock_guard<std::mutex> lock(transmit_mutex);
      transmit_buffer.insert(transmit_buffer.begin(), buffer.begin(), buffer.end());
    }
    port->closePort();
    port->openPort();
    return write_num;
  }

  if (static_cast<std::size_t>(write_num) < buffer.size()) {
    std::lock_guard<std::mutex> lock(transmit_mutex);
    transmit_buffer.insert(
      transmit_buffer.begin(), buffer.begin() + write_num, buffer.end());
  }

  return write_num;
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::SerialDriverNode)
