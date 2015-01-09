#include "ublox/ublox.h"
#include <iostream>
#include <fstream>

#include "boost/filesystem.hpp"

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
        //             12345678901234567890123456789012345678901234567890123456789012345678901234567890
        data_file_ << "%%       1) GPS Time 2) Prn  3) Pseudorange  4) Prn  5) Pseudorange ..." << std::endl;

    } catch (std::exception &e) {
        std::cout << "Error opening log file: " << e.what();
        if (data_file_.is_open())
            data_file_.close();
        return false;
    }
    return true;
}

void PseudorangeData(ublox::RawMeas raw_meas, double time_stamp) {
    try {

        data_file_ << fixed << setw(20) << setprecision(3) << (double)raw_meas.iTow;

        for(int ii=0;ii<raw_meas.numSV; ii++) {
            data_file_  << setw(20) << raw_meas.rawmeasreap[ii].svid  
                        << setw(20) << raw_meas.rawmeasreap[ii].psuedorange; // m
        }

        data_file_ << std::endl;

    } catch (std::exception &e) {
        std::cout << "RxmRawCallback() error";
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

    // Set Callback for pseudorange data
    my_gps.set_rxm_raw_callback(PseudorangeData);

    //! Configure ublox
    // request pseudorange data
    my_gps.ConfigureMessageRate(0x02,0x10,1);


    // loop forever
    while(1)
      usleep(50*1000); // sleep for 50 ms

    my_gps.Disconnect();
    StopLoggingData();
    return 0;
}
























