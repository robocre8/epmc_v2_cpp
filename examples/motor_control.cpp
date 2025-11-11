
#include <sstream>
#include <iostream>
#include <unistd.h>

#include <chrono>

#include <iomanip>

#include "epmc_v2_i2c.hpp"

EPMC_V2 epmcV2(0x55, "/dev/i2c-1");;

void delay_ms(unsigned long milliseconds)
{
  usleep(milliseconds * 1000);
}

int main(int argc, char **argv)
{

  bool sendHigh = false;

  float lowTargetVel = 0.00; // in rad/sec
  float highTargetVel = 5.00; // in rad/sec

  float pos0, pos1;
  float vel0, vel1;

  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  float sampleTime = 0.015;

  auto ctrlPrevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> ctrlDuration;
  float ctrlSampleTime = 5.0;

  for (int i=0; i<3; i+=1){
    delay_ms(1000);
    std::cout << "configuring controller: " << i+1 << " sec" << std::endl;
  }
  

  epmcV2.writeSpeed(0.0, 0.0);
  epmcV2.clearDataBuffer();

  int motor_cmd_timeout_ms = 10000;
  epmcV2.setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
  motor_cmd_timeout_ms = epmcV2.getCmdTimeout();
  std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;

  epmcV2.writeSpeed(lowTargetVel, lowTargetVel);

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
          epmcV2.writeSpeed(highTargetVel, highTargetVel);
          highTargetVel *= -1;
          sendHigh = false;
        }
        else
        {
          epmcV2.writeSpeed(lowTargetVel, lowTargetVel);
          sendHigh = true;
        }
      }
      catch(const std::exception& e)
      {
        std::cout << "Error occurred: " << e.what() << std::endl;
      }

      ctrlPrevTime = std::chrono::system_clock::now();
    }

    duration = (std::chrono::system_clock::now() - prevTime);
    if (duration.count() > sampleTime)
    {
      try
      {
        epmcV2.readMotorData(pos0, pos1, vel0, vel1);
        std::cout << "----------------------------------" << std::endl;
        std::cout << "motor0_readings: [" << pos0 << "," << vel0 << "]" << std::endl;
        std::cout << "motor1_readings: [" << pos1 << "," << vel1 << "]" << std::endl;
        std::cout << "----------------------------------" << std::endl;
        std::cout << std::endl;
      }
      catch(const std::exception& e)
      {
        std::cout << "Error occurred: " << e.what() << std::endl;
      }

      prevTime = std::chrono::system_clock::now();
    }
  }
}