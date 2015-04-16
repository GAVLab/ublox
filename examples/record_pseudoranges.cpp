#include "ublox/ublox.h"
#include <iostream>
#include <fstream>

// #include "boost/filesystem.hpp"

using namespace ublox;
using namespace std;


ofstream data_file_;  //!< file stream for logging gps data
std::string data_filename_; //!< file name for logging gps data

bool StartDataLogging(std::string filename) {
    try {

        data_filename_ = filename;
        
        // open file and add header
        data_file_.open(data_filename_.c_str());

        // write header
        data_file_ << "%%RANGE  1) GPS Time(ms) 2) SVID  3) Pseudorange (m)  4) SVID  5) Pseudorange ..." << std::endl;
        data_file_ << "%%CLOCK  1) GPS Time(ms) 2) ClockBias(nsec) 3) ClkDrift(nsec/sec) 4) TimeAccuracyEstimate(nsec) 5) FreqAccuracyEstimate(ps/s)" << std::endl;

    } catch (std::exception &e) {
        std::cout << "Error opening log file: " << e.what();
        if (data_file_.is_open())
            data_file_.close();
        return false;
    }
    std::cout << "Started Data Log" << std::endl;
    return true;
}

void PseudorangeData(ublox::RawMeas raw_meas, double time_stamp) {
    try {

        data_file_ << fixed << "RANGE" << "\t" << (signed long)raw_meas.iTow;
        for(int ii=0;ii<raw_meas.numSV; ii++) {
            data_file_  << "\t" << (unsigned int)raw_meas.rawmeasreap[ii].svid
                        << "\t" << setprecision(3) << raw_meas.rawmeasreap[ii].psuedorange; // m
        }
        data_file_ << std::endl;
        data_file_ << fixed << "CARRIER" << "\t" << (signed long)raw_meas.iTow;
        for(int ii=0;ii<raw_meas.numSV; ii++) {
            data_file_  << "\t" << (unsigned int)raw_meas.rawmeasreap[ii].svid
            << "\t" << setprecision(3) << raw_meas.rawmeasreap[ii].carrier_phase; // m
        }
        data_file_ << std::endl;
        data_file_ << fixed << "DOPPLER" << "\t" << (signed long)raw_meas.iTow;
        for(int ii=0;ii<raw_meas.numSV; ii++) {
            data_file_  << "\t" << (unsigned int)raw_meas.rawmeasreap[ii].svid
            << "\t" << setprecision(3) << raw_meas.rawmeasreap[ii].doppler; // m
        }
        data_file_ << std::endl;
        data_file_ << fixed << "CNO" << "\t" << (signed long)raw_meas.iTow;
        for(int ii=0;ii<raw_meas.numSV; ii++) {
            data_file_  << "\t" << (unsigned int)raw_meas.rawmeasreap[ii].svid
            << "\t" << setprecision(3) << (unsigned int) raw_meas.rawmeasreap[ii].cno; // m
        }
        data_file_ << std::endl;

    } catch (std::exception &e) {
        std::cout << "PseudorangeData() error";
    }    
}

// PACK(
//     struct NavClock{
//         UbloxHeader header;
//         uint32_t iTOW;
//         int32_t clkbias;    // clock bias in nanoseconds
//         int32_t clkdrift;   // clock drift in ns/s
//         uint32_t tacc;      // time accuracy estimate (ns)
//         uint32_t facc;      // frequency accuracy estimate (ps/s)
//         uint8_t checksum[2];
// });

void ClockData(ublox::NavClock nav_clock, double time_stamp) {
    try {

        data_file_  << fixed << "CLOCK" << "\t" << (signed long)nav_clock.iTOW;
        data_file_  << "\t" << (signed long) nav_clock.clkbias  // ns
                    << "\t" << (signed long) nav_clock.clkdrift // ns/s
                    << "\t" << (unsigned long) nav_clock.tacc  // ns
                    << "\t" << (unsigned long) nav_clock.facc;  // ps/s

        data_file_ << std::endl;

    } catch (std::exception &e) {
        std::cout << "ClockData() error";
    }    
}

void ParsedEphems(ublox::ParsedEphemData parsed_ephem_data, double time_stamp) {
    try{
        data_file_  << fixed << "Ephemerides" << "\t" << (signed long)parsed_ephem_data.tow;
        data_file_  << setprecision(15) << "\t" << parsed_ephem_data.prn
                    << "\t" << parsed_ephem_data.tgd
                    << "\t" << parsed_ephem_data.toc
                    << "\t" << (double) parsed_ephem_data.af0
                    << "\t" << (double) parsed_ephem_data.af1
                    << "\t" << (double) parsed_ephem_data.af2
                    << "\t" << parsed_ephem_data.anrtime
                    << "\t" << (double) parsed_ephem_data.dN
                    << "\t" << parsed_ephem_data.ecc
                    << "\t" << parsed_ephem_data.majaxis
                    << "\t" << (double) parsed_ephem_data.wo
                    << "\t" << parsed_ephem_data.ia
                    << "\t" << parsed_ephem_data.omega
                    << "\t" << parsed_ephem_data.dwo
                    << "\t" << parsed_ephem_data.dia
                    << "\t" << parsed_ephem_data.cuc
                    << "\t" << parsed_ephem_data.cus
                    << "\t" << parsed_ephem_data.crc
                    << "\t" << parsed_ephem_data.crs
                    << "\t" << parsed_ephem_data.cis
                    << "\t" << parsed_ephem_data.cic
                    << "\t" << parsed_ephem_data.toe;
        data_file_ << std::endl;
    } catch (std::exception &e) {
        std::cout << "ParsedEphems() error";
    }
    
}

void NavData(ublox::NavSol nav_data, double time_stamp) {
    try{
        data_file_  << fixed << "Position" << "\t" << (signed long) nav_data.iTOW;
        data_file_  << setprecision(12) << "\t" << nav_data.ecefX/100.
                    << "\t" << nav_data.ecefY/100.
                    << "\t" << nav_data.ecefZ/100.
                    << "\t" << nav_data.pAcc
                    << "\t" << (unsigned int)nav_data.pDop
                    << "\t" << (unsigned int)nav_data.numSV;
        data_file_ << std::endl;
        data_file_  << fixed << "Velocity" << "\t" << (signed long) nav_data.iTOW
                    << "\t" << nav_data.ecefVX/100.
                    << "\t" << nav_data.ecefVY/100.
                    << "\t" << nav_data.ecefVZ/100.
                    << "\t" << nav_data.sAcc
                    << "\t" << (unsigned int)nav_data.pDop
                    << "\t" << (unsigned int)nav_data.numSV;
        data_file_ << std::endl;

    } catch (std::exception &e) {
        std::cout << "Navigation Solution Error";
    }
    
}

void StopLoggingData() {
    if (data_file_.is_open())
        data_file_.close();
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
    //! Start Data Logging
    bool logging_on = StartDataLogging("range_data.log");
    
    my_gps.ConfigureNavigationParameters(4,2);

    // Set Callback for pseudorange data
    my_gps.set_rxm_raw_callback(PseudorangeData);
    my_gps.set_nav_clock_callback(ClockData);
    my_gps.set_parsed_ephem_callback(ParsedEphems);
    my_gps.set_nav_solution_callback(NavData);

    //! Configure ublox
    // request pseudorange data
    my_gps.ConfigureMessageRate(0x02,0x10,1);
    // nav clock data
    my_gps.ConfigureMessageRate(0x01,0x22,1);
    // nav clock data
    my_gps.ConfigureMessageRate(0x0B,0x31,1);
    // nav solution data
    my_gps.ConfigureMessageRate(0x01,0x06,1);
    
    // loop forever
    while(1)
      usleep(50*1000); // sleep for 50 ms

    my_gps.Disconnect();
    StopLoggingData();
    return 0;
}
























