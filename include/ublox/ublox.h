#ifndef UBLOX_H
#define UBLOX_H

#include "ublox_structures.h"
#include <serial/serial.h>
#include <fstream>

/*
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <cstring>
#include <iomanip>
*/

//#include <boost/function.hpp>
#include <boost/thread.hpp>
//#include <boost/bind.hpp>

namespace ublox {

typedef boost::function<double()> GetTimeCallback;
typedef boost::function<void()> HandleAcknowledgementCallback;

// Messaging callbacks
typedef boost::function<void(const std::string&)> LogMsgCallback;

// GPS Data Callbacks
typedef boost::function<void(CfgPrt&, double&)> PortSettingsCallback;
typedef boost::function<void(NavPosLLH&, double&)> NavPosLLHCallback;
typedef boost::function<void(NavSol&, double&)> NavSolCallback;
typedef boost::function<void(NavStatus&, double&)> NavStatusCallback;
typedef boost::function<void(NavVelNed&, double&)> NavVelNedCallback;
typedef boost::function<void(EphemSV&, double&)> AidEphCallback;			// Problem here
typedef boost::function<void(AlmSV&, double&)> AidAlmCallback;
typedef boost::function<void(AidHui&, double&)> AidHuiCallback;
typedef boost::function<void(AidIni&, double&)> AidIniCallback;
typedef boost::function<void(RawMeas&, double&)> RxmRawCallback;
typedef boost::function<void(SVStat&, double&)> RxmSvsiCallback;

class Ublox
{
public:
	Ublox();
	~Ublox();

	/*!
	 * Connects to the uBlox receiver given a serial port.
	 *
	 * @param port Defines which serial port to connect to in serial mode.
	 * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
	 */
	bool Connect(std::string port, int baudrate=115200);

   /*!
    * Disconnects from the serial port
    */
    void Disconnect();

    /*!
     * Pings the GPS to determine if it is properly connected
     *
     * This method sends a ping to the GPS and waits for a response.
     *
     * @param num_attempts The number of times to ping the device
     * before giving up
     * @param timeout The time in milliseconds to wait for each reponse
     *
     * @return True if the GPS was found, false if it was not.
     */
     bool Ping(int num_attempts=5);


     /*!
      * Pings the GPS to determine if it is properly connected
      *
      * This method sends a ping to the GPS and waits for a response.
      *
      * @param num_attempts The number of times to ping the device
      * before giving up
      * @param timeout The time in milliseconds to wait for each reponse
      *
      * @return True if the GPS was found, false if it was not.
      */
     void set_time_handler(GetTimeCallback time_handler) {
         this->time_handler_ = time_handler;
    }
     void SaveConfiguration();
     bool Reset(uint16_t nav_bbr_mask, uint8_t reset_mode);
     bool ResetToColdStart(uint8_t reset_mode);
     bool ResetToWarmStart();
     bool ResetToHotStart();

    void SetPortConfiguration(bool ubx_input, bool ubx_output, bool nmea_input, bool nmea_output);
    void PollPortConfiguration(uint8_t port_identifier = 3);
    bool ConfigureMessageRate(uint8_t class_id, uint8_t msg_id, uint8_t rate);

    //////////////////////////////////////////////////////
    // Diagnostic Callbacks
    //////////////////////////////////////////////////////
    LogMsgCallback log_debug_;
    LogMsgCallback log_info_;
    LogMsgCallback log_warning_;
    LogMsgCallback log_error_;

    //////////////////////////////////////////////////////
    // Data Callbacks
    //////////////////////////////////////////////////////
    HandleAcknowledgementCallback handle_acknowledgement_;
    GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping

    //////////////////////////////////////////////////////
    // Aiding Data Polling Messages
    //////////////////////////////////////////////////////
    bool PollMessage(uint8_t class_id, uint8_t msg_id);
    bool PollMessageIndSV(uint8_t class_id, uint8_t msg_id, uint8_t svid);
    bool PollEphem(int8_t svid = -1);
    bool PollAlmanac(int8_t svid = -1);
    bool PollHUI();
    bool PollIniAid();
    bool PollAllAidData();
    bool PollRawDgpsData();
    bool PollSVStatus();
    bool PollNavStatus();

//////////////////////////////////////////////////////
// Saving/Reading stored data
//////////////////////////////////////////////////////
    bool SaveEphemerides();
    Ephemerides LoadEphemerides();
    bool SaveAlmanac();
    Almanac LoadAlmanac();
//////////////////////////////////////////////////////
// Send Aiding Data to Receiver
//////////////////////////////////////////////////////
    bool SendMessage(uint8_t *msg_ptr, size_t length);
    bool SendAidIni(AidIni ini);
    bool SendAidEphem(Ephemerides ephems);
    bool SendRawMeas();
    bool SendAidHui();
    bool SendAidAlm(Almanac almanac);

    void set_aid_eph_callback(AidEphCallback callback){aid_eph_callback_=callback;};
    void set_aid_hui_callback(AidHuiCallback callback){aid_hui_callback_=callback;};
    void set_aid_ini_callback(AidIniCallback callback){aid_ini_callback_=callback;};
    void set_nav_status_callback(NavStatusCallback callback){nav_status_callback_=callback;};
    void set_nav_solution_callback(NavSolCallback callback){nav_sol_callback_=callback;};
    void set_get_time_callback(GetTimeCallback callback){time_handler_=callback;};


// Temporary Method
    Ephemerides stored_ephems;
    Almanac stored_almanac;
    Ephemerides cur_ephemerides;    // Contains newest ephemeris available for all 32 SVs
    Almanac cur_almanac;            // Contains almanac data available for all 32 SVs
    NavStatus cur_nav_status;
    NavSol cur_nav_sol;
    NavVelNed cur_nav_vel_ned;
    NavPosLLH cur_nav_position;
    AidHui cur_aid_hui;
    AidIni cur_aid_ini;
    RawMeas cur_raw_meas;
    SVStat cur_sv_stat;
    CfgPrt cur_port_settings;
private:

	/*!
	 * Starts a thread to continuously read from the serial port.
	 *
	 * Starts a thread that runs 'ReadSerialPort' which constatly reads
	 * from the serial port.  When valid data is received, parse and then
	 *  the data callback functions are called.
	 *
	 * @see xbow440::DataCallback, xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StopReading
	 */
	void StartReading();

	/*!
	 * Starts the thread that reads from the serial port
	 *
	 * @see xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StartReading
	 */
	void StopReading();

	/*!
	 * Method run in a seperate thread that continuously reads from the
	 * serial port.  When a complete packet is received, the parse
	 * method is called to process the data
	 *
	 * @see xbow440::XBOW440::Parse, xbow440::XBOW440::StartReading, xbow440::XBOW440::StopReading
	 */
	void ReadSerialPort();

    bool RequestLogOnChanged(std::string log); //!< request the given log from the receiver at the given rate
	bool WaitForAck(int timeout); //!< waits for an ack from receiver (timeout in seconds)

    void BufferIncomingData(uint8_t* msg, size_t length);
	//! Function to parse logs into a usable structure
    void ParseLog(uint8_t* log, size_t logID);
	//! Function to parse out useful ephemeris parameters
    ParsedEphemData Parse_aid_eph(EphemSV ubx_eph);


    //////////////////////////////////////////////////////
    // Serial port reading members
    //////////////////////////////////////////////////////
	//! Serial port object for communicating with sensor
	serial::Serial *serial_port_;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;  //!< True if the read thread is running, false otherwise.


    //////////////////////////////////////////////////////
    // New Data Callbacks
    //////////////////////////////////////////////////////
    PortSettingsCallback port_settings_callback_;
    NavPosLLHCallback nav_pos_llh_callback_;
	NavSolCallback nav_sol_callback_;
    NavStatusCallback nav_status_callback_;
    NavVelNedCallback nav_vel_ned_callback_;
    AidAlmCallback aid_alm_callback_;
    AidEphCallback aid_eph_callback_;
    AidHuiCallback aid_hui_callback_;
    AidIniCallback aid_ini_callback_;
    RxmRawCallback rxm_raw_callback_;
    RxmSvsiCallback rxm_svsi_callback_;
	
	//////////////////////////////////////////////////////
	// Incoming data buffers
	//////////////////////////////////////////////////////
	unsigned char data_buffer_[MAX_NOUT_SIZE];	//!< data currently being buffered to read
	unsigned char* data_read_;		//!< used only in BufferIncomingData - declared here for speed
	size_t bytes_remaining_;	//!< bytes remaining to be read in the current message
	size_t buffer_index_;		//!< index into data_buffer_
	size_t header_length_;	//!< length of the current header being read
	bool reading_acknowledgement_;	//!< true if an acknowledgement is being received
	double read_timestamp_; 		//!< time stamp when last serial port read completed
	double parse_timestamp_;		//!< time stamp when last parse began
	unsigned short msgID;
	
    void calculateCheckSum(uint8_t* in, size_t length, uint8_t* out);

};
}

#endif
