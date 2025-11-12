
#include <sstream>
#include <iostream>
#include <unistd.h>

#include <chrono>

#include <iomanip>

#include "epmc_v2_i2c.hpp"

EPMC_V2 epmcV2;

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
  auto currentTime = std::chrono::system_clock::now();

  for (int i=0; i<3; i+=1){
    delay_ms(1000);
    std::cout << "configuring controller: " << i+1 << " sec" << std::endl;
  }
  
  epmcV2.connect(0x55);
  epmcV2.writeSpeed(0.0, 0.0);
  epmcV2.clearDataBuffer();

  int motor_cmd_timeout_ms = 10000;
  epmcV2.setCmdTimeout(motor_cmd_timeout_ms); // set motor command timeout
  motor_cmd_timeout_ms = epmcV2.getCmdTimeout();
  std::cout << "motor command timeout: " << motor_cmd_timeout_ms << " ms" << std::endl;

  sendHigh = true;

  prevTime = std::chrono::system_clock::now();

  while (true)
  {
    currentTime = std::chrono::system_clock::now();
    try{
      epmcV2.writeSpeed(highTargetVel, highTargetVel);
      epmcV2.readMotorData(pos0, pos1, vel0, vel1);

      duration = currentTime - prevTime;
      std::cout << "duration: " << duration.count() << std::endl;
    }
    catch(const std::exception& e)
    {
      std::cout << "Error occurred: " << e.what() << std::endl;
    }
    prevTime = currentTime;
    
  }
    
}