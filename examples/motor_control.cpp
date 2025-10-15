
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
  float sampleTime = 0.02;

  auto ctrlPrevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> ctrlDuration;
  float ctrlSampleTime = 4.0;

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

  epmcV2.writeSpeed(lowTargetVel, lowTargetVel, lowTargetVel, lowTargetVel);

  sendHigh = true;

  prevTime = std::chrono::system_clock::now();
  ctrlPrevTime = std::chrono::system_clock::now();

  while (true)
  {

    ctrlDuration = (std::chrono::system_clock::now() - ctrlPrevTime);
    if (ctrlDuration.count() > ctrlSampleTime)
    {
      try
      {
        if (sendHigh)
        {
          epmcV2.writeSpeed(highTargetVel, highTargetVel, highTargetVel, highTargetVel);

          sendHigh = false;
        }
        else
        {
          epmcV2.writeSpeed(lowTargetVel, lowTargetVel, lowTargetVel, lowTargetVel);

          sendHigh = true;
        }
      }
      catch(const std::exception& e)
      {
        std::cout << "Error occurred: ";
      }

      ctrlPrevTime = std::chrono::system_clock::now();
    }

    duration = (std::chrono::system_clock::now() - prevTime);
    if (duration.count() > sampleTime)
    {
      try
      {
        bool success = epmcV2.readMotorData(pos0, pos1, pos2, pos3, vel0, vel1, vel2, vel3);

        if (success){
         
        }
        std::cout << "----------------------------------" << std::endl;
        std::cout << "motor0_readings: [" << pos0 << std::fixed << std::setprecision(4) << "," << vel0 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << "motor1_readings: [" << pos1 << std::fixed << std::setprecision(4) << "," << vel1 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << "motor2_readings: [" << pos2 << std::fixed << std::setprecision(4) << "," << vel2 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << "motor3_readings: [" << pos3 << std::fixed << std::setprecision(4) << "," << vel3 << std::fixed << std::setprecision(4) << "]" << std::endl;
        std::cout << "----------------------------------" << std::endl;
        std::cout << std::endl;
      }
      catch (int e)
      {
        std::cout << "Error occurred: ";
      }

      prevTime = std::chrono::system_clock::now();
    }
  }
}