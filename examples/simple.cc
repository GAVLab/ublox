#include <serial/serial.h>
#include <iostream>
#include <string>

#include <termios.h>
#include <fcntl.h>

using serial::Serial;
using serial::Timeout;
using std::string;

void printHex(char *data, int length)
{
    for(int i = 0; i < length; ++i) {
        printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
    }
    printf("\n");
}

void printStuff(uint8_t *buffer, ssize_t bytes_read) {
        std::cout << "Read " << bytes_read << " bytes." << std::endl;
        printHex((char*)buffer, bytes_read);
        char latitude[4];
        latitude[0]=buffer[14];
        latitude[1]=buffer[15];
        latitude[2]=buffer[16];
        latitude[3]=buffer[17];
        printHex(latitude,4);
        int32_t latitude_int;
        memcpy(&latitude_int,latitude,4);
        std::cout << "latitude: " << (double)latitude_int*1e-7 << std::endl;
        char longitude[4];
        longitude[0]=buffer[10];
        longitude[1]=buffer[11];
        longitude[2]=buffer[12];
        longitude[3]=buffer[13];
        printHex(longitude,4);
        int32_t longitude_int;
        memcpy(&longitude_int,longitude,4);
        std::cout << "longitude: " << (double)longitude_int*1e-7 << std::endl;
}

void serial_lib(char* port) {
    Timeout t(100, 1000, 0, 1000, 0);
    Serial s(port, 9600, t);

     while (1) {
        uint8_t buffer[5000];
        size_t bytes_read = s.read(buffer, 5000);
        if (bytes_read<18)
            continue;
        printStuff(buffer, bytes_read);
     }
}

void unix_lib(char* port) {
    int fd_ = ::open (port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ == -1) {
        printf("Error opening serial port.\n");
        return;
    }
    struct termios options; // The options for the file descriptor

    if (tcgetattr(fd_, &options) == -1) {
        printf("Error getting serial port settings.\n");
    }

    ::cfsetispeed(&options, B9600);
    ::cfsetospeed(&options, B9600);

    options.c_cc[VMIN] = 36;
    options.c_cc[VTIME] = 10;

    ::tcsetattr (fd_, TCSANOW, &options);

    while (1) {
        uint8_t buffer[5000];
        ssize_t bytes_read = ::read(fd_, buffer, 5000);
        if (bytes_read<18)
            continue;
	printStuff(buffer, bytes_read);
    }
}

int main(int argc, char** argv) {
    if (argc != 3) {
	printf("Usage: simple <serial port> <1|2>\n");
    }
    if (string(argv[2]) == "1") {
        serial_lib(argv[1]);
    }
    if (string(argv[2]) == "2") {
        unix_lib(argv[1]);
    }

    return 0;
}

