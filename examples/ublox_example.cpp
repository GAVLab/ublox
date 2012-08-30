#include "ublox/ublox.h"
#include <iostream>
using namespace ublox;
using namespace std;


// Global Variables
Almanac stored_almanac;
NavStatus cur_nav_status;
AidIni cur_aid_ini;
double aid_ini_timestamp;


int main(int argc, char **argv)
{
    Ublox my_gps;
    double ttff_unassisted;
    double ttff_assisted;

    if(argc < 3) {
        std::cerr << "Usage: ublox_example <serial port address> <baud rate>" << std::endl;
        return 0;
    }
    std::string port(argv[1]);
    int baudrate=115200;
    istringstream(argv[2]) >> baudrate;

    // Connect to Receiver
    bool result = my_gps.Connect(port,baudrate);
        if (result) {
            cout << "Successfully connected." << endl;
        }
        else {
            cout << "Failed to connect." << endl;
            return -1;
        }

    // request position message
    my_gps.ConfigureMessageRate(0x01,0x03,1);

    // loop forever
    while(1)
      usleep(50*1000); // sleep for 50 ms

    my_gps.Disconnect();

    return 0;
}
























