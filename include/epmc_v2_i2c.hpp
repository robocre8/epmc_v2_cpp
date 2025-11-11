#ifndef EPMC_V2_I2C_HPP
#define EPMC_V2_I2C_HPP

#include <cstdint>
#include <string>
#include <cmath> // Required for std::round, std::ceil, std::floor
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <i2c/i2c.h>

double round_to_dp(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::round(value * multiplier) / multiplier;
}

class EPMC_V2
{
public:
    explicit EPMC_V2(int slave_addr, const std::string &device = "/dev/i2c-1");
    ~EPMC_V2();

    void writeSpeed(float v0, float v1);
    void writePWM(int pwm0, int pwm1);
    void readPos(float &pos0, float &pos1);
    void readVel(float &v0, float &v1);
    void readUVel(float &v0, float &v1);
    bool setCmdTimeout(int timeout_ms);
    int getCmdTimeout();
    bool setPidMode(int mode);
    int getPidMode();
    bool clearDataBuffer();
    void readMotorData(float &pos0, float &pos1, float &v0, float &v1);


private:
    int fd;
    int slaveAddr;
    void send_packet_without_payload(uint8_t cmd);
    void write_data1(uint8_t cmd, uint8_t pos, float val);
    void write_data2(uint8_t cmd, float val0, float val1);
    void read_data1(float &val0);
    void read_data2(float &val0, float &val1);
    void read_data4(float &val0, float &val1, float &val2, float &val3);

    //  Protocol Command IDs -------------
    const uint8_t START_BYTE = 0xAA;
    const uint8_t WRITE_VEL = 0x01;
    const uint8_t WRITE_PWM = 0x02;
    const uint8_t READ_POS = 0x03;
    const uint8_t READ_VEL = 0x04;
    const uint8_t READ_UVEL = 0x05;
    const uint8_t SET_PID_MODE = 0x15;
    const uint8_t GET_PID_MODE = 0x16;
    const uint8_t SET_CMD_TIMEOUT = 0x17;
    const uint8_t GET_CMD_TIMEOUT = 0x18;
    const uint8_t READ_MOTOR_DATA = 0x2A;
    const uint8_t CLEAR_DATA_BUFFER = 0x2C;
    //---------------------------------------------
};





EPMC_V2::EPMC_V2(int slave_addr, const std::string &device) : slaveAddr(slave_addr)
{
    fd = i2c_open(device.c_str());
    if (fd < 0) {
        perror("Failed to open I2C device");
        exit(1);
    }
    if (i2c_set_slave(fd, slaveAddr) < 0) {
        perror("Failed to set I2C slave address");
        exit(1);
    }
}

EPMC_V2::~EPMC_V2()
{
    if (fd >= 0)
        i2c_close(fd);
}

uint8_t EPMC_V2::computeChecksum(uint8_t *packet, uint8_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++)
        sum += packet[i];
    return sum & 0xFF;
}

void EPMC_V2::send_packet_without_payload(uint8_t cmd)
{
    // Build packet: start_byte + cmd + length + pos + float + checksum
    uint8_t packet[4];
    packet[0] = START_BYTE;
    packet[1] = cmd;
    packet[2] = 0; // msg length = 0

    // Compute checksum
    uint8_t checksum = computeChecksum(packet, 3);
    packet[3] = checksum;

    if (i2c_write(fd, packet, sizeof(packet)) != sizeof(packet))
        perror("I2C send_packet_without_payload failed");
    usleep(5000);
}

void EPMC_V2::write_data1(uint8_t cmd, uint8_t pos, float val)
{
    // Build packet: start_byte + cmd + length + pos + float + checksum
    uint8_t packet[1 + 1 + 1 + 1 + 4 + 1];
    packet[0] = START_BYTE;
    packet[1] = cmd;
    packet[2] = 5; // msg is uint8 + float = 5byte length
    packet[3] = pos;
    memcpy(&packet[4], &val, sizeof(float));

    // Compute checksum
    uint8_t checksum = computeChecksum(packet, 8);
    packet[8] = checksum;

    if (i2c_write(fd, packet, sizeof(packet)) != sizeof(packet))
        perror("I2C write_data1 failed");
    usleep(5000);
}

void EPMC_V2::write_data2(uint8_t cmd, float val0, float val1)
{
    // Build packet: start_byte + cmd + length + float*2 + checksum
    uint8_t packet[1 + 1 + 1 + 8 + 1];
    packet[0] = START_BYTE;
    packet[1] = cmd;
    packet[2] = 8; // msg is 2 float = 8byte length
    memcpy(&packet[3], &val0, sizeof(float));
    memcpy(&packet[7], &val1, sizeof(float));
    // Compute checksum
    uint8_t checksum = computeChecksum(packet, 11);
    packet[11] = checksum;

    if (i2c_write(fd, packet, sizeof(packet)) != sizeof(packet))
        perror("I2C write_data3 failed");
    usleep(5000);
}

void EPMC_V2::read_data1(float &val0)
{
    uint8_t buffer[4];
    if (i2c_read(fd, buffer, 4) != 4) {
        perror("I2C read_data1 failed");
        val0 = 0.0f;
    }
    std::memcpy(&val0, buffer[0], 4);
}

void EPMC_V2::read_data2(float &val0, float &val1)
{
    uint8_t buffer[8];
    if (i2c_read(fd, buffer, 8) != 8) {
        perror("I2C read_data3 failed");
        val0 = val1 = 0.0f;
        return;
    }
    std::memcpy(&val0, buffer[0], 4);
    std::memcpy(&val1, buffer[4], 4);
}

void EPMC_V2::read_data4(float &val0, float &val1, float &val2, float &val3)
{
    uint8_t buffer[16];
    if (i2c_read(fd, buffer, 16) != 16) {
        perror("I2C read_data3 failed");
        val0 = val1 = val2 = val3 = 0.0f;
        return;
    }
    std::memcpy(&val0, buffer[0], 4);
    std::memcpy(&val1, buffer[4], 4);
    std::memcpy(&val0, buffer[8], 4);
    std::memcpy(&val1, buffer[12], 4);
}

void EPMC_V2::writeSpeed(float v0, float v1){
  write_data2(WRITE_VEL, v0, v1);
}

void EPMC_V2::writePWM(int pwm0, int pwm1){
  write_data2(WRITE_VEL, (float)pwm0, (float)pwm1);
}

void EPMC_V2::readPos(float &pos0, float &pos1){
  send_packet_without_payload(READ_POS);
  read_data2(pos0, pos1);
}

void EPMC_V2::readVel(float &v0, float &v1){
  send_packet_without_payload(READ_VEL);
  read_data2(v0, v1);
}

void EPMC_V2::readUVel(float &v0, float &v1){
  send_packet_without_payload(READ_UVEL);
  read_data2(v0, v1);
}

void EPMC_V2::readMotorData(float &pos0, float &pos1, float &v0, float &v1){
  send_packet_without_payload(READ_MOTOR_DATA);
  read_data4(pos0, pos1, v0, v1);
}

bool EPMC_V2::setCmdTimeout(int timeout_ms){
  float res;
  write_data1(SET_CMD_TIMEOUT, 100, (float)timeout_ms);
  read_data1(res);
  return ((int)res == 1) ? true : false;
}

int EPMC_V2::getCmdTimeout(){
  float timeout_ms;
  write_data1(GET_CMD_TIMEOUT, 100, 0.0);
  read_data1(timeout_ms);
  return (int)timeout_ms;
}

bool EPMC_V2::setPidMode(int mode){
  float res;
  write_data1(SET_PID_MODE, 100, (float)mode);
  read_data1(res);
  return ((int)res == 1) ? true : false;
}

int EPMC_V2::getPidMode(){
  float mode;
  write_data1(GET_PID_MODE, 100, 0.0);
  read_data1(mode);
  return (int)mode;
}

bool EPMC_V2::clearDataBuffer(){
  float res;
  write_data1(CLEAR_DATA_BUFFER, 100, 0.0);
  read_data1(res);
  return ((int)res == 1) ? true : false;
}


#endif
