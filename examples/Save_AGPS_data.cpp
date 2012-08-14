//#include <string>
//#include <iostream>
//#include <sstream>
#include <bitset>
#include <fstream>
#include "ublox/ublox.h"
using namespace ublox;
using namespace std;

// Global Variables
Almanac stored_almanac;
AidHui cur_aid_hui;
NavStatus cur_nav_status;

//////////////////////////////////////////////////////////////
// Save/Read data to file
//////////////////////////////////////////////////////////////
bool SaveHUI()
{
    // Serializing struct to HUI.data
        ofstream output_file("HUI.data", ios::binary);
        output_file.write((char*)&cur_aid_hui, sizeof(cur_aid_hui));
        output_file.close();
        return true;
}

bool SaveAlmanac()
{
    // Serializing struct to Almanac.data
        ofstream output_file("Almanac.data", ios::binary);
        output_file.write((char*)&stored_almanac, sizeof(stored_almanac));
        output_file.close();
        return true;
}

double GetTime() {
    boost::posix_time::ptime present_time(boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return duration.total_seconds();
}

void NavigationStatusCallback(NavStatus &status, double &time_stamp) {
    cur_nav_status = status;
}

void AlmanacCallback(AlmSV &almanac, double &time_stamp) {
    stored_almanac.almsv[almanac.svprn] = almanac;
    if (almanac.header.payload_length==40)
        cout << "[" << time_stamp <<  "]" <<  "Received almanac for sv " << almanac.svprn << endl;
    else
        cout << "[" << time_stamp <<  "]" <<  "No almanac available for sv " << almanac.svprn << endl;
}

void HuiCallback(AidHui &hui, double &time_stamp) {
    cur_aid_hui = hui;
    std::bitset<32> svhealth(hui.health);
    std::cout << dec << "[" << time_stamp << "]" << "Received aid_hui." << std::endl;
    std::cout << "SV health flags: " << svhealth << std::endl;
    std::cout << "TOW: " << hui.tow << std::endl;
    std::cout << "Week #: " << hui.week << std::endl;
    std::cout << "Leap Seconds: " << hui.beforeleapsecs << std::endl;
}


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

// Set Port Configuration
    //my_gps.SetPortConfiguration(0, 0, 0, 0);

// Setup Callbacks
    my_gps.set_nav_status_callback(NavigationStatusCallback);
    my_gps.set_aid_alm_callback(AlmanacCallback);
    my_gps.set_aid_hui_callback(HuiCallback);

    my_gps.ConfigureMessageRate(0x01,0x03,1); // nav status at 1 Hz

// Make sure receiver gets a fix
   while (cur_nav_status.fixtype !=0x03) // wait for 3D fix
       usleep(200*1000);

   cout << "3D fix obtained." << endl;
   cout << " TTFF: " << (cur_nav_status.ttff/1000.) << " sec" << endl;
   cout << " Time since startup: " << (cur_nav_status.msss/1000.) << endl << endl;

// Reset stored almanac data
   memset(&stored_almanac, 0, sizeof(stored_almanac));
   memset(&cur_aid_hui, 0, sizeof(cur_aid_hui));


// Poll almanac data
   my_gps.PollAlmanac();

// Poll HUI data
   my_gps.PollHUI();


// Wait to get almanac data for all SVs (assuming #32 is sent last)
   while(stored_almanac.almsv[32].header.sync1==0)
       usleep(200*1000);

// make sure we get HUI
   while(cur_aid_hui.header.sync1==0)
       usleep(200*1000);

// Save Almanac and HUI to File

    std::cout << "Saving..." << endl;
    SaveAlmanac();
    SaveHUI();
    usleep(5000*1000);


// Disconnect receiver
    std::cout << "Disconnecting from receiver." << endl;
    my_gps.Disconnect();

    return 0;
}
