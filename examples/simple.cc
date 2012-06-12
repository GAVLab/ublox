#include <serial/serial.h>
#include <iostream>

using serial::Serial;
using serial::Timeout;

void printHex(char *data, int length)
{
    for(int i = 0; i < length; ++i){
        printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
    }
    printf("\n");
}


int main(void) {
    Timeout t(100, 1000, 0, 1000, 0);
    Serial s("/dev/ttyACM0", 9600, t);

    while (1) {
        uint8_t buffer[5000];
        size_t bytes_read = s.read(buffer, 5000);
        printHex((char*)buffer, bytes_read);
        if (bytes_read<18)
            continue;
        char latitude[4];
        latitude[0]=buffer[14];
        latitude[1]=buffer[15];
        latitude[2]=buffer[16];
        latitude[3]=buffer[17];
	printHex(latitude,4);
        int32_t latitude_int;
        memcpy(&latitude_int,latitude,4);
        std::cout << "latitude: " << latitude_int << std::endl;
        char longitude[4];
        longitude[0]=buffer[10];
        longitude[1]=buffer[11];
        longitude[2]=buffer[12];
        longitude[3]=buffer[13];
	printHex(longitude,4);
        int32_t longitude_int;
        memcpy(&longitude_int,longitude,4);
        std::cout << "longitude: " << longitude_int << std::endl;
   }

    return 0;
}

