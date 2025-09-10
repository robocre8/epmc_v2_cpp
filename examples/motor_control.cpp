
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

  float lowTargetVel = 0.00; // in rad/sec
  float highTargetVel = 3.142; // in rad/sec

  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  float sampleTime = 0.02;

  auto ctrlPrevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> ctrlDuration;
  float ctrlSampleTime = 4.0;

  // std::string port = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.4:1.0-port0";
  std::string port = "/dev/ttyUSB0";
  epmcV2.connect(port);

  delay_ms(2000);

  // left wheels (motor 0 and motor 2)
  // right wheels (motor 1 and motor 3)
  epmcV2.writeSpeed(0.0, 0.0, 0.0, 0.0);

  int motor_cmd_timeout_ms = 0;
  epmcV2.setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
  motor_cmd_timeout_ms = epmcV2.getCmdTimeout();
  std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;

  // left wheels (motor 0 and motor 2)
  // right wheels (motor 1 and motor 3)
  epmcV2.writeSpeed(lowTargetVel, lowTargetVel, lowTargetVel, lowTargetVel);

  sendHigh = true;

  prevTime = std::chrono::system_clock::now();
  ctrlPrevTime = std::chrono::system_clock::now();

  while (true)
  {

    ctrlDuration = (std::chrono::system_clock::now() - ctrlPrevTime);
    if (ctrlDuration.count() > ctrlSampleTime)
    {
      if (sendHigh)
      {
        // left wheels (motor 0 and motor 2)
        // right wheels (motor 1 and motor 3)
        epmcV2.writeSpeed(highTargetVel, highTargetVel, highTargetVel, highTargetVel);

        sendHigh = false;
      }
      else
      {
        // left wheels (motor 0 and motor 2)
        // right wheels (motor 1 and motor 3)
        epmcV2.writeSpeed(lowTargetVel, lowTargetVel, lowTargetVel, lowTargetVel);

        sendHigh = true;
      }

      ctrlPrevTime = std::chrono::system_clock::now();
    }

    duration = (std::chrono::system_clock::now() - prevTime);
    if (duration.count() > sampleTime)
    {
      try
      {
        // left wheels (motor 0 and motor 2)
        // right wheels (motor 1 and motor 3)
        float pos0, pos1, pos2, pos3;
        float v0, v1, v2, v3;
        epmcV2.readPos(pos0, pos1, pos2, pos3);
        epmcV2.readVel(v0, v1, v2, v3);

        std::cout << "----------------------------------" << std::endl;
        std::cout << "left wheels - motor 0 and motor 2" << std::endl;
        std::cout << "motor0_readings: [" << pos0 << std::fixed << std::setprecision(4) << "," << v0 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << "motor2_readings: [" << pos2 << std::fixed << std::setprecision(4) << "," << v2 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << std::endl;
        std::cout << "right wheels - motor 1 and motor 3" << std::endl;
        std::cout << "motor1_readings: [" << pos1 << std::fixed << std::setprecision(4) << "," << v1 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << "motor3_readings: [" << pos3 << std::fixed << std::setprecision(4) << "," << v3 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << "----------------------------------" << std::endl;
        std::cout << std::endl;
      }
      catch (...)
      {
        
      }

      prevTime = std::chrono::system_clock::now();
    }
  }
}