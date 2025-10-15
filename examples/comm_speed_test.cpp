
#include <sstream>
#include <iostream>
#include <unistd.h>

#include <chrono>

#include <iomanip>

#include "epmc_v2.hpp"

EPMC_V2 epmcV2;

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

int main(int argc, char **argv)
{

  bool sendHigh = false;

  float lowTargetVel = -10.00; // in rad/sec
  float highTargetVel = 10.00; // in rad/sec

  float pos0, pos1, pos2, pos3;
  float vel0, vel1, vel2, vel3;

  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  auto currentTime = std::chrono::system_clock::now();

  // std::string port = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0";
  std::string port = "/dev/ttyUSB0";
  epmcV2.connect(port);

  for (int i=0; i<4; i+=1){
    delay_ms(1000);
    std::cout << "configuring controller: " << i+1 << " sec" << std::endl;
  }
  

  epmcV2.writeSpeed(0.0, 0.0, 0.0, 0.0);
  epmcV2.clearDataBuffer();

  int motor_cmd_timeout_ms = 5000;
  epmcV2.setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
  motor_cmd_timeout_ms = epmcV2.getCmdTimeout();
  std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;

  sendHigh = true;

  prevTime = std::chrono::system_clock::now();

  while (true)
  {
    currentTime = std::chrono::system_clock::now();

    epmcV2.writeSpeed(highTargetVel, highTargetVel, highTargetVel, highTargetVel);
    bool success = epmcV2.readMotorData(pos0, pos1, pos2, pos3, vel0, vel1, vel2, vel3);

    if (success){

    }

    duration = currentTime - prevTime;
    std::cout << "duration: " << duration.count() << std::endl;

    prevTime = currentTime;
  }
    
}