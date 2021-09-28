//
// Created by yenkn on 2020/9/27.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <proto/hal/tiny_serial.h>
#include <TinyProtocol.h>
#include <thread>
#include <functional>

#define MSG_TYPE_TWIST 0xe1
#define MSG_TYPE_PID 0xe2

struct TwistMessage {
  int16_t velocity;
  int16_t angle;
};

struct PIDMessage {
  float kp;
  float ki;
};


typedef unsigned char byte;

int16_t generate_message(byte target, byte type, const byte *data, int16_t len, byte *out) {
  int16_t pos = 0, dpos = 0;

  out[pos++] = target;
  out[pos++] = type;

  while(dpos < len) {
    out[pos++] = data[dpos++];
  }

  return pos;
}

tiny_serial_handle_t port_handle;
Tiny::ProtoLight proto;

void command_callback(const geometry_msgs::TwistConstPtr &msg) {
  double steering = std::max(std::min(-msg->angular.z / M_PI * 180, 30.0), -30.0); // 转换成下位机需要的角度，并进行限幅
  double throttle = std::max(std::min(msg->linear.x * 100, 25.0), -25.0);

  // 构造消息数据
  TwistMessage twist = {
      int16_t (throttle),
      int16_t (steering),
  };

  byte buf[100] = { 0 };
  int16_t size = generate_message(0, MSG_TYPE_TWIST, (byte *)&twist, sizeof(TwistMessage), buf);
  proto.write(reinterpret_cast<char *>(buf), size);
}

int serial_send_fd(void *p, const void *buf, int len) {
  return tiny_serial_send(port_handle, buf, len);
}

int serial_receive_fd(void *p, void *buf, int len) {
  return tiny_serial_read(port_handle, buf, len);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arduino_example_node");
  ros::NodeHandle nh;

  // port, baudrate, timeout in milliseconds
  port_handle = tiny_serial_open("/dev/ttyUSB0", 57600);
  if(port_handle == TINY_SERIAL_INVALID) {
    std::cerr << "Error opening serial port" << std::endl;
    return 1;
  }
  proto.enableCheckSum();
  proto.begin(serial_send_fd, serial_receive_fd);

  ros::Subscriber cmd_subscriber = nh.subscribe("/cmd_vel", 10, command_callback);

  ros::spin();
  proto.end();

  tiny_serial_close(port_handle);
  return 0;
}