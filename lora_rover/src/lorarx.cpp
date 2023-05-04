/****************************************************************************/
//   Description: This code receives specified buffer through termios serial
//                library and extracts meaningful values.
//   Created On: 21/04/2023
//   Created By: Alperen Demirkol
/****************************************************************************/


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
    tty.c_cc[VTIME] = 0;
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
        
        tcflush(fd, TCIFLUSH);
        ssize_t n = read(fd, buffer, sizeof(buffer)); // read a packet

        printf("%02X", buffer[0]);
        printf("%02X", buffer[1]);
        printf("%02X", buffer[2]);
        printf("%02X", buffer[3]);
        printf("%02X", buffer[4]);
        printf("\n");

        // Check the newline character
        if (buffer[5] != '\n')
        {
            std::cerr << "Error: newline character not found" << std::endl;
            continue;
        }

        // Extract packet data
        Packet packet;
        memcpy(&packet, buffer, sizeof(Packet));

        packet.value1 = (int8_t)buffer[0];
        packet.value2 = (int8_t)buffer[1];
        packet.value3 = (uint8_t) (buffer[2] & 0x0F);
        packet.value4 = (uint8_t) ((buffer[2] & 0xF0) >> 4);

        packet.crc = ((uint16_t)buffer[3] << 8) | (uint16_t)buffer[4];

        //calculated crc
        uint16_t crc = crc16(reinterpret_cast<uint8_t*>(&packet), sizeof(packet) - 3);

        // Verify CRC
        if (crc != packet.crc)
        {
            std::cerr << "CRC mismatch" << std::endl;
            continue;
        }

        else 
        {
            printf("Received Packet: %d %d %d %d\n", packet.value1, packet.value2, packet.value3, packet.value4);
        }
    }
}
