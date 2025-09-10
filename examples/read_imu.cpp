
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
  auto prevTime = std::chrono::system_clock::now();
  std::chrono::duration<double> duration;
  float sampleTime = 0.02;

  std::string port = "/dev/ttyUSB0";
  epmcV2.connect(port);

  // wait for the epmcV2 to fully setup
  for (int i = 1; i <= 2; i += 1)
  {
    delay_ms(1000);
    std::cout << "configuring controller: " << i << " sec" << std::endl;
  }

  int use_imu = epmcV2.getUseIMU();

  prevTime = std::chrono::system_clock::now();

  while (true)
  {
    duration = (std::chrono::system_clock::now() - prevTime);
    if (duration.count() > sampleTime)
    {
      try
      {
        if (use_imu == 1){
          float ax, ay, az;
          float gx, gy, gz;
          epmcV2.readAcc(ax, ay, az);
          epmcV2.readGyro(gx, gy, gz);

          std::cout << "ax: " << ax << std::fixed << std::setprecision(4);
          std::cout << "\tay: " << ay << std::fixed << std::setprecision(4);
          std::cout << "\taz: " << az << std::fixed << std::setprecision(4) << std::endl;
          std::cout << "gx: " << gx << std::fixed << std::setprecision(4);
          std::cout << "\tgy: " << gy << std::fixed << std::setprecision(4);
          std::cout << "\tgz: " << gz << std::fixed << std::setprecision(4) << std::endl;
          std::cout << std::endl;
        }
        else{
          std::cout << "IMU Mode Not Activated" << std::endl;
        }
      }
      catch (...)
      {
      }

      prevTime = std::chrono::system_clock::now();
    }
  }
}