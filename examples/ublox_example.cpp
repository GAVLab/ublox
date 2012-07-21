//#include <string>
//#include <iostream>
//#include <sstream>
//#include <fstream>
#include "ublox/ublox.h"
#include <iostream>
using namespace ublox;
using namespace std;


//void ProcessData(const ImuData& data) {
//    cout << "Received data. ax: " << data.ax << " ay: " <<
//        data.ay << " az: " << data.az << std::endl;
//
//};


int main(int argc, char **argv)
{
    Ublox my_gps;

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

    // Run Receiver until I have all aiding data

        // Set Port Configuration
            //my_gps.SetPortConfiguration(1, 1, 0, 0);
        // Set NAV-STATUS message to output every second
            //my_gps.ConfigureMessageRate(0x01,0x03,0);
            //sleep(1); std::cout << "pause 1 sec" << endl;
        // Poll for all aiding data

            my_gps.PollIniAid();
            sleep(1); std::cout << "pause 1 sec" << endl;
                // Receiver Pos, Frequecny, Time

/*
            my_gps.PollEphem();
            sleep(1);  std::cout << "pause 1 sec" << endl;

            my_gps.PollHUI();
            sleep(1); std::cout << "pause 1 sec" << endl;
                // Health, UTC, Ionosphere

            my_gps.PollAlmanac();
            sleep(1); std::cout << "pause 1 sec" << endl;
*/
           // my_gps.ConfigureMessageRate(0x01,0x03,0);
            //sleep(1); std::cout << "pause 1 sec" << endl;

    // Reset to Cold Start
       //my_gps.ResetToColdStart(0x02);
       //sleep(1); std::cout << "pause 1 sec" << endl;    // Max of 100 ms needed to process AID-INI

    // Pass all aiding data to receiver
/*
           my_gps.SendAidIni();
           sleep(.1); std::cout << "pause 1 sec" << endl;    // Max of 100 ms needed to process AID-INI


           my_gps.SendAidEphem();
           sleep(1); std::cout << "pause 1 sec" << endl;

           my_gps.SendAidHui();
           sleep(1); std::cout << "pause 1 sec" << endl;
           my_gps.SendAidAlm();
*/
           // Poll for all aiding data
/*
               my_gps.PollIniAid();
               sleep(1); std::cout << "pause 1 sec" << endl;
                   // Receiver Pos, Frequecny, Time


               my_gps.PollEphem();
               sleep(1);  std::cout << "pause 1 sec" << endl;

               my_gps.PollHUI();
               sleep(1); std::cout << "pause 1 sec" << endl;
                   // Health, UTC, Ionosphere

               my_gps.PollAlmanac();
               sleep(1); std::cout << "pause 1 sec" << endl;
*/

my_gps.ConfigureMessageRate(0x01,0x03,1);


       // my_gps.ResetToHotStart();
    // Log TTFF

    // Reset to Cold Start
        //my_gps.ResetToColdStart(0x02);
        //sleep(1); std::cout << "pause 1 sec" << endl;


    // Log TTFF

    while(1);

        my_gps.Disconnect();

    return 0;
}
























