#include <string>
#include <iostream>
#include <sstream>

#include "ublox/ublox.h"
using namespace ublox;
using namespace std;


//void ProcessData(const ImuData& data) {
//    cout << "Received data. ax: " << data.ax << " ay: " <<
//        data.ay << " az: " << data.az << std::endl;
//
//};

int main(int argc, char **argv)
{
    if(argc < 3) {
        std::cerr << "Usage: ublox_example <serial port address> <baud rate>" << std::endl;
        return 0;
    }
    std::string port(argv[1]);
    int baudrate=115200;
    istringstream(argv[2]) >> baudrate;


    Ublox my_gps;
    bool result = my_gps.Connect(port,baudrate);

    if (result) {
        cout << "Successfully connected." << endl;
    }
    else {
        cout << "Failed to connect." << endl;
        return -1;
    }

    // configure receiver
    // request NAVPOSLLH message
    //my_gps.ConfigureMessageRate(0x01, 0x02, 1);
	//Request AID_EPH
     my_gps.PollEphem();

    while(1);

    my_gps.Disconnect();
		

    // Send message to clear all data on receiver

        // log time it takes to get a fix
    // Collect and save aiding data
    // Reset receiver and clear all data
    // Share aiding data
        // log TTFF

    return 0;
}


























