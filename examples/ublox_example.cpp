#include "ublox/ublox.h"
#include <iostream>
using namespace ublox;
using namespace std;


<<<<<<< HEAD
=======
//void ProcessData(const ImuData& data) {
//    cout << "Received data. ax: " << data.ax << " ay: " <<
//        data.ay << " az: " << data.az << std::endl;
//
//};

// Global Variables
Almanac stored_almanac;
NavStatus cur_nav_status;
AidIni cur_aid_ini;
double aid_ini_timestamp;

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

void PositionTimeCallback(AidIni &init_position, double &time_stamp) {
    cur_aid_ini = init_position;
    aid_ini_timestamp = time_stamp;
    cout << dec << "[" << time_stamp <<  "]" <<  "Received aid_ini." << endl;
    cout << "Pos X: " << init_position.ecefXorLat << endl;
    cout << "Pos Y: " << init_position.ecefYorLon << endl;
    cout << "Pos Z: " << init_position.ecefZorAlt << endl;
    cout << "TOW1 = " << init_position.time_of_week << " ms" << endl;
    cout << "TOW2 = " << init_position.time_of_week_ns << " ns" << endl;
    cout << "Time accuracy1 = " << init_position.time_accuracy_ms << " ms" << endl;
    cout << "Time accuracy2 = " << init_position.time_accuracy_ns << " ns" << endl;
    cout << "Flags: 0x" << hex << (int)init_position.flags << dec << endl<<endl;
}


>>>>>>> chris
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

<<<<<<< HEAD
    // request position message
    my_gps.ConfigureMessageRate(0x01,0x03,1);

    // loop forever
    while(1)
      usleep(50*1000); // sleep for 50 ms

=======
// Set Port Configuration
    //my_gps.SetPortConfiguration(0, 0, 0, 0);


// Setup Callbacks
    my_gps.set_nav_status_callback(NavigationStatusCallback);
    my_gps.set_aid_alm_callback(AlmanacCallback);
    my_gps.set_aid_ini_callback(PositionTimeCallback);

    my_gps.ConfigureMessageRate(0x01,0x03,1); // nav status at 1 Hz

//   my_gps.PollRawDgpsData();

//////////////////////////////////////////
// Reset to Hot Start
//////////////////////////////////////////
  // my_gps.ResetToHotStart();

   //while (cur_nav_status.fixtype !=0x00) // wait for receiver to reset
     //  usleep(200*1000);

   cout << "Receiver reset. Waiting for unassisted fix" << endl;

   while (cur_nav_status.fixtype !=0x03) // wait for 3D fix
       usleep(200*1000);

   cout << "3D fix obtained." << endl;
   cout << " TTFF: " << (cur_nav_status.ttff/1000.) << " sec" << endl;
   cout << " Time since startup: " << (cur_nav_status.msss/1000.) << endl << endl;
   ttff_unassisted = cur_nav_status.ttff/1000.;

   // Reset stored almanac data
   memset(&stored_almanac, 0, sizeof(stored_almanac));
   memset(&cur_aid_ini, 0, sizeof(cur_aid_ini));

   // request aiding data from receiver
   my_gps.PollIniAid();  // poll position and time
   usleep(100*1000);

   // Poll almanac data
   my_gps.PollAlmanac();

   // make sure we got the aid_ini data
   while(cur_aid_ini.header.sync1==0)
       usleep(200*1000);

   // Wait to get almanac data for all SVs (assuming #32 is sent last)
   while(stored_almanac.almsv[32].header.sync1==0)
       usleep(200*1000);

   ////////////////////////////////////////////
   // Reset Receiver
   ////////////////////////////////////////////
   cout << "Aiding data stored. Resetting receiver." << endl;
   my_gps.ResetToColdStart(0x02);

   while (cur_nav_status.fixtype !=0x00) // wait for receiver to reset
       usleep(200*1000);

   cout << "Receiver reset. Waiting for assisted fix" << endl;

   ///////////////////////////////////////////////////////////////////
   // SEND AIDING DATA
   // update time in AidIni message and send
   cur_aid_ini.flags = cur_aid_ini.flags & 0xF7; // clear time pulse flag
   cur_aid_ini.time_accuracy_ms=1000;
   double cur_time = GetTime();
   int time_correction=(cur_time-aid_ini_timestamp)*1000;
   std::cout << "cur_time = " << cur_time << "sec" << std::endl;
   std::cout << "aid_ini_timestamp = " << aid_ini_timestamp << "sec" << std::endl;
   cout << "Correct time by " << (cur_time-aid_ini_timestamp) << " sec." << endl;
   cout << "Correct time by " << time_correction << " ms." << endl;
   cur_aid_ini.time_of_week=cur_aid_ini.time_of_week + time_correction;
   cout << "Initialize receiver position and time." << endl;

   // send aid-ini
   my_gps.SendAidIni(cur_aid_ini);

   // send almanac
   cout << "Send almanac back to receiver." << endl;
   my_gps.SendAidAlm(stored_almanac);

   //my_gps.PollAlmanac();

   // WAIT FOR FIX
   cout << "Wait for assisted fix." << endl;

   while (cur_nav_status.fixtype !=0x03) // wait for 3D fix
       usleep(200*1000);

   cout << "3D fix obtained." << endl;
   cout << " TTFF: " << (cur_nav_status.ttff/1000.) << " sec" << endl;
   cout << " Time since startup: " << (cur_nav_status.msss/1000.) << endl << endl;
   ttff_assisted = cur_nav_status.ttff/1000.;

   memset(&stored_almanac, 0, sizeof(stored_almanac));
   my_gps.PollAlmanac();
   // Wait to get almanac data for all SVs (assuming #32 is sent last)
   while(stored_almanac.almsv[32].header.sync1==0)
       usleep(200*1000);

    std::cout << "Disconnecting from receiver." << endl;
>>>>>>> chris
    my_gps.Disconnect();

    return 0;
}
























