#include <ros/ros.h>
#include <lostation_msgs/lostation.h>
#include <iostream>
#include <exception>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE B9600
#define PACKET_SIZE 6

using namespace std;
lostation_msgs::lostation cmd;

struct Packet 
{
    int8_t value1;
    int8_t value2;
    uint8_t value3;
    uint8_t value4;
    uint16_t crc;
};

// CRC-16 polynomial: x^16 + x^15 + x^2 + 1 (0x8005)
uint16_t crc16(uint8_t *data, size_t length) 
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) 
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) 
        {
            if (crc & 0x8000) 
            {
                crc = (crc << 1) ^ 0x8005;
            } 
            else 
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void station_cmd_cb(const lostation_msgs::lostation::ConstPtr& msg)
{
    cmd = *msg;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "ground_station_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Subscriber sub_station = nh.subscribe("/lora/commands", 1, station_cmd_cb);

    // Open the serial port
    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) 
    {
        std::cerr << "Error opening serial port\n";
        return 1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &options);

    while(ros::ok())
    {
        try 
        {
            // double cmd_x = cmd.cmd_x.data;
            // double cmd_yaw = cmd.cmd_yaw.data;
            // int target_gps = cmd.target_gps.data;
            // std::string curr_status = cmd.curr_status.data;

            // Fill packet
            Packet packet = {0};
            packet.value1 = static_cast<int>(std::round(std::abs(cmd.cmd_x.data) * 100)) % 100;
            if (cmd.cmd_x.data < 0) 
                packet.value1 *= -1;
            packet.value2 = static_cast<int>(std::round(std::abs(cmd.cmd_yaw.data) * 100)) % 100;
            if (cmd.cmd_yaw.data < 0) 
                packet.value2 *= -1;
            packet.value3 = cmd.target_gps.data;
            packet.value4 = cmd.curr_status.data;
            packet.crc = crc16(reinterpret_cast<uint8_t*>(&packet), sizeof(packet) - 1);

            // Fill Buffer
            uint8_t buffer[PACKET_SIZE] = {0};
            buffer[0] = (uint8_t)packet.value1;
            buffer[1] = (uint8_t)packet.value2;
            buffer[2] = ((uint8_t)packet.value3 & 0x0F) | (((uint8_t)packet.value4 & 0x0F) << 4);
            buffer[3] = ((uint8_t)packet.crc >> 8) & 0xFF;
            buffer[4] = (uint8_t)packet.crc & 0xFF;
            buffer[5] = '\n';

            /*
            Buffer format:
            Byte 0: 8-bit integer value1
            Byte 1: 8-bit integer value2
            Byte 2: 4 bits from integer value3, followed by 4 bits from integer value4
            Byte 3: Most significant 8 bits of the 16-bit CRC
            Byte 4: Least significant 8 bits of the 16-bit CRC
            Byte 5: Newline character ('\n')
            */

            // printf("Buffer: ");
            // for (int i = 0; i < sizeof(buffer); i++) 
            // {
            //     printf("%02X", buffer[i]);
            // }

            // printf("\n");

            write(fd, buffer, sizeof(buffer));
            //usleep(100000);  // wait 100ms
        }

        catch (const exception& e) 
        {
            cerr << "Caught exception: " << e.what() << "\n";
            return 1;
        }
        ros::spinOnce();
        loop_rate.sleep();
}

    close(fd);
    return 0;
}
