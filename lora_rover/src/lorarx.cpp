#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

#define BAUDRATE B9600
#define DEVICE "/dev/ttyUSB0"

// Define packet struct
#pragma pack(push, 1)
typedef struct {
    int8_t value1;
    int8_t value2;
    uint8_t value3;
    uint8_t value4;
    uint16_t crc;
    uint8_t newline;
} Packet;
#pragma pack(pop)

// Function to calculate CRC16
uint16_t crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (size_t j = 0; j < 8; ++j)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

int main()
{
    int fd;
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    tcsetattr(fd, TCSANOW, &tty);
    fd = open(DEVICE, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
        std::cerr << "Error opening " << DEVICE << std::endl;
        return 1;
    }

    while (true)
    {
        uint8_t buffer[sizeof(Packet)];
        ssize_t n = read(fd, buffer, sizeof(buffer)); // read a packet

        if (n == sizeof(buffer))
        {
            // Check the newline character
            if (buffer[sizeof(Packet)-1] != '\n')
            {
                std::cerr << "Error: newline character not found" << std::endl;
                continue;
            }

            // Extract packet data
            Packet packet;
            memcpy(&packet, buffer, sizeof(Packet));

            // Extract packet data and CRC
            uint8_t packet_data[4];
            uint16_t packet_crc = 0;
            for (int i = 0; i < 6; i++) 
            {
                if (i < 4) 
                {
                    packet_data[i] = buffer[i];
                } else if (i < 6) 
                {
                    packet_crc |= (uint16_t)buffer[i] << ((i-4)*8);
                }
            }

            // Verify CRC
            uint16_t crc = crc16(buffer, sizeof(Packet)-3);
            if (crc != packet.crc)
            {
                std::cerr << "CRC mismatch" << std::endl;
                continue;
            }

            packet.value1 = (int8_t)buffer[0];
            packet.value2 = (int8_t)buffer[1];
            packet.value3 = (uint8_t) (buffer[2] & 0x0F);
            packet.value4 = (uint8_t) ((buffer[2] & 0xF0) >> 4);
            printf("Received Packet: %d %d %d %d\n", packet.value1, packet.value2, packet.value3, packet.value4);
        }
    }
}
