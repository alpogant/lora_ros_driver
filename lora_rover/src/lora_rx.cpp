#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <chrono>
#include <thread>

#define SERIAL_PORT "/dev/ttyUSB1" 
#define BAUD_RATE B9600

int main(int argc, char** argv)
{
    using namespace std::this_thread;
    using namespace std::chrono_literals;
    using std::chrono::system_clock;

    int serial_port;
    struct termios tty;
    unsigned int value;

    serial_port = open(SERIAL_PORT, O_RDWR);
    if (serial_port < 0) {
        perror("Failed to open serial port");
        exit(-1);
    }

    memset(&tty, 0, sizeof(tty));
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tcsetattr(serial_port, TCSANOW, &tty);

    while (1) {
        int n = read(serial_port, &value, sizeof(value));
        if (n > 0) {
            printf("%d Received value: %u\n", n, value);
        }
        sleep_for(1000ms);
    }

    close(serial_port);

    return 0;
}
