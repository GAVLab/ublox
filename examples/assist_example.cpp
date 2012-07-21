//#include <string>
//#include <iostream>
//#include <sstream>
//#include <fstream>
#include "ublox/ublox.h"
#include <iostream>
using namespace ublox;
using namespace std;


// global variables
Ephemerides stored_ephems;
AidIni cur_aid_ini;
double aid_ini_timestamp;
NavStatus cur_nav_status;

double GetTime() {
    boost::posix_time::ptime present_time(boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return duration.total_seconds();
}

void EphemerisCallback(EphemSV &ephemeris, double &time_stamp) {
    stored_ephems.ephemsv[ephemeris.svprn] = ephemeris;

    if (ephemeris.header.payload_length==104)
        cout << "[" << time_stamp <<  "]" <<  "Received ephemeris for sv " << ephemeris.svprn << endl;
    else
        cout << "[" << time_stamp <<  "]" <<  "No ephemeris available for sv " << ephemeris.svprn << endl;
}

void PositionTimeCallback(AidIni &init_position, double &time_stamp) {
    cur_aid_ini = init_position;
    aid_ini_timestamp = time_stamp;
    //timeval t1;
    //gettimeofday(&t1, NULL);
    //cout << dec << "[" << time_stamp <<  "]" <<  "Received aid_ini." << endl;
    cout << "Pos X: " << init_position.ecefXorLat << endl;
    cout << "Pos Y: " << init_position.ecefYorLon << endl;
    cout << "Pos Z: " << init_position.ecefZorAlt << endl;
    cout << "TOW1 = " << init_position.time_of_week << " ms" << endl;
    cout << "TOW2 = " << init_position.time_of_week_ns << " ns" << endl;
    cout << "Time accuracy1 = " << init_position.time_accuracy_ms << " ms" << endl;
    cout << "Time accuracy2 = " << init_position.time_accuracy_ns << " ns" << endl;
    cout << "Flags: 0x" << hex << (int)init_position.flags << dec << endl<<endl;
}

void NavigationStatusCallback(NavStatus &status, double &time_stamp) {
    cur_nav_status = status;
}


int main(int argc, char **argv)
{
    Ublox my_gps;
    double ttff_unassisted;
    double ttff_assisted;

    if(argc < 3) {
        std::cerr << "Usage: assist_example <serial port address> <baud rate>" << std::endl;
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

    // setup callbacks
    my_gps.set_aid_eph_callback(EphemerisCallback);
    my_gps.set_aid_ini_callback(PositionTimeCallback);
    my_gps.set_nav_status_callback(NavigationStatusCallback);

    // turn off nmea messages
    //my_gps.SetPortConfiguration();

    // request nav status data and wait for fix
    my_gps.ConfigureMessageRate(0x01,0x03,1); // nav status at 1 Hz

    ///////////////////////////////////////////////////////////////////////////
    // RESET RECEIVER AND PERFORM UNASSISTED COLD START                      //
    ///////////////////////////////////////////////////////////////////////////

    // reset receiver
    cout << "Resetting receiver." << endl;
    my_gps.ResetToColdStart(0x02);

    while (cur_nav_status.fixtype !=0x00) // wait for receiver to reset
        usleep(200*1000);

    cout << "Receiver reset. Waiting for unassisted fix" << endl;

    while (cur_nav_status.fixtype !=0x03) // wait for 3D fix
        usleep(200*1000);

    cout << "3D fix obtained." << endl;
    cout << " TTFF: " << (cur_nav_status.ttff/1000.) << " sec" << endl;
    cout << " Time since startup: " << (cur_nav_status.msss/1000.) << endl << endl;
    ttff_unassisted = cur_nav_status.ttff/1000.;

    // clear stored assist data
    memset(&stored_ephems, 0, sizeof(stored_ephems));
    memset(&cur_aid_ini, 0, sizeof(cur_aid_ini));

    // request aiding data from receiver
    my_gps.PollIniAid();  // poll position and time
    usleep(100*1000);
    my_gps.PollEphem(); // poll ephemeris for all satellites

    // make sure we got the aid_ini data
    while(cur_aid_ini.header.sync1==0)
        usleep(200*1000);
    // make sure we got all of the ephemeris
    while(stored_ephems.ephemsv[32].header.sync1==0)
        usleep(200*1000);


    ///////////////////////////////////////////////////////////////////////////
    // RESET RECEIVER AND PERFORM ASSISTED COLD START                        //
    ///////////////////////////////////////////////////////////////////////////

    // reset receiver
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
    my_gps.SendAidIni(cur_aid_ini);

    // send ephemeris
    cout << "Send ephemeris back to receiver." << endl;
    my_gps.SendAidEphem(stored_ephems);

    ///////////////////////////////////////////////////////////////////
    // WAIT FOR FIX
    cout << "Wait for assisted fix." << endl;

    while (cur_nav_status.fixtype !=0x03) // wait for 3D fix
        usleep(200*1000);

    cout << "3D fix obtained." << endl;
    cout << " TTFF: " << (cur_nav_status.ttff/1000.) << " sec" << endl;
    cout << " Time since startup: " << (cur_nav_status.msss/1000.) << endl << endl;
    ttff_assisted = cur_nav_status.ttff/1000.;

    ///////////////////////////////////////////////////////////////////
    // DISPLAY RESULTS

    cout << endl << endl << "Results:" << endl;
    cout << " Unassisted TTFF (sec): " << ttff_unassisted << endl;
    cout << " Assisted TTFF (sec): " << ttff_assisted << endl;

    my_gps.Disconnect();

    return 0;
}
























