#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

#define PACKET_SIZE 6

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

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <serial_port>" << std::endl;
        return 1;
    }

    // Open the serial port
    int serial_fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd == -1) {
        std::cerr << "Failed to open serial port" << std::endl;
        return 1;
    }

    // Configure the serial port
    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    tcsetattr(serial_fd, TCSANOW, &options);

    // Receive and process data packets
    uint8_t buffer[PACKET_SIZE];
    uint16_t expected_crc, actual_crc;

    while (true) 
    {
        int bytes_received = read(serial_fd, buffer, PACKET_SIZE);
        if (bytes_received == PACKET_SIZE) 
        {
            // Calculate the expected CRC
            expected_crc = (uint16_t)buffer[4] << 8 | (uint16_t)buffer[5];

            // Calculate the actual CRC of the data
            actual_crc = crc16(buffer, 4);

            // Verify the CRC
            if (expected_crc != actual_crc) 
            {
                std::cerr << "ERROR: CRC mismatch" << std::endl;
                continue;
            }

            // Extract the data from the buffer
            uint8_t value1 = buffer[0];
            uint8_t value2 = buffer[1];
            uint8_t value3 = buffer[2];
            uint8_t value4 = buffer[3];

            // Print the data to the terminal
            std::cout << "Received: " << static_cast<unsigned>(value1) << ", "
                      << static_cast<unsigned>(value2) << ", "
                      << static_cast<unsigned>(value3) << ", "
                      << static_cast<unsigned>(value4) << ", ";
        }
    }
}
