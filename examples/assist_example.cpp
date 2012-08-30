#include "ublox/ublox.h"
#include <iostream>
#include <bitset>
#include <fstream>
using namespace ublox;
using namespace std;


// global variables
AidIni cur_aid_ini;
Ephemerides stored_ephems;
Almanac stored_almanac;
AidHui cur_aid_hui;
double aid_ini_timestamp;
NavStatus cur_nav_status;

inline void printHex(char *data, int length)
{
    for(int i = 0; i < length; ++i){
        printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
    }
    printf("\n");
}

AidHui LoadHUI(){
    AidHui hui;
    ifstream input_file("HUI.data", ios::binary);
    input_file.read((char*)&hui, sizeof(hui));
    std::cout << "Loading HUI from file..." << std::endl;
    printHex((char*) &hui, sizeof(hui));
    return hui;
}

Almanac LoadAlmanac(){
    ifstream input_file("Almanac.data", ios::binary);
    input_file.read((char*)&stored_almanac, sizeof(stored_almanac));
    std::cout << "Loading Almanac from file..." << std::endl;
    printHex((char*) &stored_almanac, sizeof(stored_almanac));
    return stored_almanac;

}


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
    double ttff_unassisted;
    double ttff_assisted;
    bool postime_onoff;
    bool ephem_onoff;
    bool alm_onoff;
    bool hui_onoff;

    if(argc < 3) {
        std::cerr << "Usage: Full_AGPS_assist <serial port address> <baud rate>" << std::endl;
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
    my_gps.set_aid_alm_callback(AlmanacCallback);
    my_gps.set_aid_hui_callback(HuiCallback);
    my_gps.set_aid_ini_callback(PositionTimeCallback);
    my_gps.set_nav_status_callback(NavigationStatusCallback);

    // turn off nmea messages
    //my_gps.SetPortConfiguration();

    // request nav status data and wait for fix
    my_gps.ConfigureMessageRate(0x01,0x03,1); // nav status at 1 Hz

    // Loop 10 times to get average TTFF
    uint8_t iterations = 10;
    double un_ttff[iterations];
    double as_ttff[iterations];

    // Turn AGPS data on/off
    postime_onoff = 1;
    ephem_onoff = 0;
    alm_onoff = 0;
    hui_onoff = 0;

    for (uint8_t i=0; i<iterations; i++)
    {

    ///////////////////////////////////////////////////////////////////////////
    // Perform Unassisted Cold Start                                         //
    ///////////////////////////////////////////////////////////////////////////

    // reset receiver
    cout << "Resetting receiver." << endl;
    my_gps.ResetToColdStart(0x02);

    while (cur_nav_status.fixtype !=0x00) // wait for receiver to reset
        usleep(200*1000);

    cout << "Receiver reset. Waiting for unassisted fix" << endl;

    // clear stored assist data
    memset(&cur_aid_ini, 0, sizeof(cur_aid_ini));
    memset(&stored_ephems, 0, sizeof(stored_ephems));
    memset(&stored_almanac, 0, sizeof(stored_almanac));
    memset(&cur_aid_hui, 0, sizeof(cur_aid_hui));

    while (cur_nav_status.fixtype !=0x03) // wait for 3D fix
        usleep(200*1000);

    usleep(2000*1000);

    cout << "3D fix obtained." << endl;
    cout << " TTFF: " << (cur_nav_status.ttff/1000.) << " sec" << endl;
    cout << " Time since startup: " << (cur_nav_status.msss/1000.) << endl << endl;
    ttff_unassisted = cur_nav_status.ttff/1000.;

    // All AGPS data present, request aiding data from receiver
    if(postime_onoff == 1){
    my_gps.PollIniAid();  // poll position and time
    usleep(100*1000);
    }
    if(ephem_onoff == 1){
    my_gps.PollEphem(); // poll ephemeris for all satellites
    usleep(100*1000);
    }

    if(alm_onoff == 1){
        stored_almanac = LoadAlmanac();
    }
    if(hui_onoff == 1){
        cur_aid_hui = LoadHUI();
    }
    usleep(3000*1000);
    // make sure we got the aid_ini data
    if(postime_onoff == 1){
    while(cur_aid_ini.header.sync1==0)
        usleep(200*1000);
    }
    // make sure we got all of the ephemeris
    if(ephem_onoff == 1){
    while(stored_ephems.ephemsv[32].header.sync1==0)
        usleep(200*1000);
    }
    // Wait to get almanac data for all SVs (assuming #32 is sent last)
    if(alm_onoff == 1){
    while(stored_almanac.almsv[32].header.sync1==0)
        usleep(200*1000);
    }
    // make sure we get HUI
    if(hui_onoff == 1){
    while(cur_aid_hui.header.sync1==0)
        usleep(200*1000);
    }

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
    if(postime_onoff == 1){
    cur_aid_ini.flags = cur_aid_ini.flags & 0xF7; // clear time pulse flag
    cur_aid_ini.flags = cur_aid_ini.flags & 0xFB; // clear clock drift flag
    cur_aid_ini.flags = cur_aid_ini.flags & 0xFE; // clear clock freq flag
    cur_aid_ini.clock_drift_or_freq=0;

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
    }
    // send ephemeris
    if(ephem_onoff == 1){
    cout << "Send ephemeris back to receiver." << endl;
    my_gps.SendAidEphem(stored_ephems);
    }

    // send almanac
    if(alm_onoff == 1){
    cout << "Send almanac back to receiver." << endl;
    my_gps.SendAidAlm(stored_almanac);
    }
    // send HUI
    if(hui_onoff == 1){
    cout << "Send HUI back to receiver." << endl;
    my_gps.SendAidHui(cur_aid_hui);
    }

    ///////////////////////////////////////////////////////////////////
    // WAIT FOR FIX
    cout << "Wait for assisted fix." << endl;

    while (cur_nav_status.fixtype !=0x03) // wait for 3D fix
        usleep(200*1000);

    cout << "3D fix obtained." << endl;
    cout << " TTFF: " << (cur_nav_status.ttff/1000.) << " sec" << endl;
    cout << " Time since startup: " << (cur_nav_status.msss/1000.) << endl << endl;
    ttff_assisted = cur_nav_status.ttff/1000.;

    if(alm_onoff == 1){
    std::cout << "Verify that Almanac was stored on receiver" << endl;
    // Poll Almanac to see if SendAlm worked
    memset(&stored_almanac, 0, sizeof(stored_almanac));
    my_gps.PollAlmanac();
    while(stored_almanac.almsv[32].header.sync1==0)
        usleep(200*1000);
    }

    if(hui_onoff == 1){
    std::cout << "Verify that HUI was stored on receiver" << endl;
    // Poll Aid-HUI to see if SendAidHui worked
    memset(&cur_aid_hui, 0, sizeof(cur_aid_hui));
    my_gps.PollHUI();
    while(cur_aid_hui.header.sync1==0)
        usleep(200*1000);
    }
    ///////////////////////////////////////////////////////////////////
    // DISPLAY RESULTS

    cout << endl << endl << "Results:" << endl;
    cout << " Unassisted TTFF (sec): " << ttff_unassisted << endl;
    cout << " Assisted TTFF (sec): " << ttff_assisted << endl;

    // Store Unassisted and Assisted TTFF Iterations
    un_ttff[i] = ttff_unassisted;
    as_ttff[i] = ttff_assisted;

    }

    // Average TTFF Iterations
    double un_total = 0;
    double as_total = 0;


    for (uint8_t j=0; j< iterations; j++)
    {
        un_total = un_total + un_ttff[j];
        as_total = as_total + as_ttff[j];
        //std::cout << "un_total = " << un_total << endl;
        //std::cout << "as_total = " << as_total << endl;
        std::cout << "un_ttff = " << un_ttff << endl;
        std::cout << "as_ttff = " << as_ttff << endl;

    }

    double un_ave_ttff = un_total/iterations;
    double as_ave_ttff = as_total/iterations;
    //std::cout << "Results for Complete AGPS assist" << endl;
    //std::cout << "Results for Complete AGPS with degraded AGPS time accuracy" << endl;
    std::cout << "Location: Stadium Parking Deck" << endl;
    std::cout << "Scenario: Under Overhang" << endl;
    std::cout << "Aiding used: AID-INI" << endl;
    //std::cout << "Results for INI and ephem AGPS with degraded AGPS time accuracy" << endl;
    std::cout << "Average Unassisted TTFF = " << un_ave_ttff << endl;
    std::cout << "Average Assisted TTFF = " << as_ave_ttff << endl;

    my_gps.Disconnect();

    return 0;
}
























