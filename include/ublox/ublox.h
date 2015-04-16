/*!
 * \file ublox/ublox.h
 * \author  David Hodo <david.hodo@gmail.com>
 * \author  Chris Collins <cnc0003@tigermail.auburn.edu>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2012 IS4S / Auburn University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a cross platform interface for uBlox GPS receivers using
 * ths UBX protocol.
 */

#ifndef UBLOX_H
#define UBLOX_H

#include "ublox_structures.h"
#include <serial/serial.h>
#include <fstream>

#include <boost/function.hpp>
#include <boost/thread.hpp>
//#include <boost/bind.hpp>

namespace ublox {

typedef boost::function<double()> GetTimeCallback;
typedef boost::function<void()> HandleAcknowledgementCallback;

// Messaging callbacks
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarningMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;

// GPS Data Callbacks
typedef boost::function<void(CfgPrt&, double&)> PortSettingsCallback;
typedef boost::function<void(CfgNav5&, double&)> ConfigureNavigationParametersCallback;
typedef boost::function<void(NavPosLLH&, double&)> NavPosLLHCallback;
typedef boost::function<void(NavSol&, double&)> NavSolCallback;
typedef boost::function<void(NavStatus&, double&)> NavStatusCallback;
typedef boost::function<void(NavVelNed&, double&)> NavVelNedCallback;
typedef boost::function<void(NavSVInfo&, double&)> NavSVInfoCallback;
typedef boost::function<void(NavGPSTime&, double&)> NavGPSTimeCallback;
typedef boost::function<void(NavUTCTime&, double&)> NavUTCTimeCallback;
typedef boost::function<void(NavDOP&, double&)> NavDOPCallback;
typedef boost::function<void(NavDGPS&, double&)> NavDGPSCallback;
typedef boost::function<void(NavClock&, double&)> NavClockCallback;
typedef boost::function<void(EphemSV&, double&)> AidEphCallback;
typedef boost::function<void(AlmSV&, double&)> AidAlmCallback;
typedef boost::function<void(AidHui&, double&)> AidHuiCallback;
typedef boost::function<void(AidIni&, double&)> AidIniCallback;
typedef boost::function<void(RawMeas&, double&)> RxmRawCallback;
typedef boost::function<void(SubframeData&, double&)> RxmSubframeCallback;
typedef boost::function<void(SVStatus&, double&)> RxmSvsiCallback;
typedef boost::function<void(ParsedEphemData&, double&)> ParsedEphemCallback;

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

    //! Indicates if a connection to the receiver has been established.
    bool IsConnected() {return is_connected_;}

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

     void set_time_handler(GetTimeCallback time_handler) {
         this->time_handler_ = time_handler;
    }
     void SaveConfiguration(); // TODO: Implement this
     bool Reset(uint16_t nav_bbr_mask, uint8_t reset_mode);
     bool ResetToColdStart(uint8_t reset_mode);
     bool ResetToWarmStart();
     bool ResetToHotStart();

    void SetPortConfiguration(bool ubx_input, bool ubx_output, bool nmea_input, bool nmea_output);
    bool ConfigureMessageRate(uint8_t class_id, uint8_t msg_id, uint8_t rate);
        // (rate) is relative to the event a message is registered on. For example,
        // if the rate of a navigation message is set to 2, the message is sent
        // every second navigation solution.

    bool ConfigureNavigationParameters(uint8_t dynamic_model = 3, uint8_t fix_mode = 3);
    bool SbasOff(); //< Tell ublox not to use SBAS SVs
    bool SbasOn(); //< Tell ublox to use SBAS for corrections, integritiy monitoring, and ranging

    //////////////////////////////////////////////////////
    // Diagnostic Callbacks
    //////////////////////////////////////////////////////
    DebugMsgCallback log_debug_;
    InfoMsgCallback log_info_;
    WarningMsgCallback log_warning_;
    ErrorMsgCallback log_error_;

    //////////////////////////////////////////////////////
    // Data Callbacks
    //////////////////////////////////////////////////////
    HandleAcknowledgementCallback handle_acknowledgement_;
    GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping

    //////////////////////////////////////////////////////
    // Receiver Configuration Settings Polling Messages
    //////////////////////////////////////////////////////
    void PollPortConfiguration(uint8_t port_identifier = 3);
    bool PollNavigationParamterConfiguration();

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
    bool PollSVInfo();
    bool PollNavStatus();

//////////////////////////////////////////////////////
// Send Aiding Data to Receiver
//////////////////////////////////////////////////////
    bool SendMessage(uint8_t *msg_ptr, size_t length);
    bool SendAidIni(AidIni ini);
    bool SendAidEphem(Ephemerides ephems);
    bool SendRawMeas(RawMeas raw_meas);
    bool SendAidHui(AidHui hui);
    bool SendAidAlm(Almanac almanac);

    void set_rxm_svsi_callback(RxmSvsiCallback callback){rxm_svsi_callback_=callback;};
    void set_rxm_subframe_callback(RxmSubframeCallback callback){rxm_subframe_callback_=callback;};
    void set_rxm_raw_callback(RxmRawCallback callback){rxm_raw_callback_=callback;};
    void set_aid_alm_callback(AidAlmCallback callback){aid_alm_callback_=callback;};
    void set_aid_eph_callback(AidEphCallback callback){aid_eph_callback_=callback;};
    void set_aid_hui_callback(AidHuiCallback callback){aid_hui_callback_=callback;};
    void set_aid_ini_callback(AidIniCallback callback){aid_ini_callback_=callback;};
    void set_nav_status_callback(NavStatusCallback callback){nav_status_callback_=callback;};
    void set_nav_solution_callback(NavSolCallback callback){nav_sol_callback_=callback;};
    void set_nav_position_llh_callback(NavPosLLHCallback callback){nav_pos_llh_callback_=callback;};
    void set_nav_sv_info_callback(NavSVInfoCallback callback){nav_sv_info_callback_=callback;};
    void set_nav_gps_time_callback(NavGPSTimeCallback callback){nav_gps_time_callback_=callback;};
    void set_nav_utc_time_callback(NavUTCTimeCallback callback){nav_utc_time_callback_=callback;};
    void set_nav_dop_callback(NavDOPCallback callback){nav_dop_callback_=callback;};
    void set_nav_dgps_callback(NavDGPSCallback callback){nav_dgps_callback_=callback;};
    void set_nav_clock_callback(NavClockCallback callback){nav_clock_callback_=callback;};
    void set_get_time_callback(GetTimeCallback callback){time_handler_=callback;};
    void set_nav_vel_ned_callback(NavVelNedCallback callback){nav_vel_ned_callback_=callback;};
    void set_port_settings_callback(PortSettingsCallback callback){port_settings_callback_ =callback;};
    void set_configure_navigation_parameters_callback(ConfigureNavigationParametersCallback callback){
        configure_navigation_parameters_callback_ = callback;};
    void set_parsed_ephem_callback(ParsedEphemCallback callback){parsed_ephem_callback_ = callback;};

    void calculateCheckSum(uint8_t* in, size_t length, uint8_t* out);
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
    ConfigureNavigationParametersCallback configure_navigation_parameters_callback_;
    NavPosLLHCallback nav_pos_llh_callback_;
    NavSolCallback nav_sol_callback_;
    NavStatusCallback nav_status_callback_;
    NavVelNedCallback nav_vel_ned_callback_;
    NavSVInfoCallback nav_sv_info_callback_;
    NavGPSTimeCallback nav_gps_time_callback_;
    NavUTCTimeCallback nav_utc_time_callback_;
    NavDOPCallback nav_dop_callback_;
    NavDGPSCallback nav_dgps_callback_;
    NavClockCallback nav_clock_callback_;
    AidAlmCallback aid_alm_callback_;
    AidEphCallback aid_eph_callback_;
    AidHuiCallback aid_hui_callback_;
    AidIniCallback aid_ini_callback_;
    RxmRawCallback rxm_raw_callback_;
    RxmSubframeCallback rxm_subframe_callback_;
    RxmSvsiCallback rxm_svsi_callback_;
    ParsedEphemCallback parsed_ephem_callback_;
	
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
	
  bool is_connected_; //!< indicates if a connection to the receiver has been established

    

};
}

#endif
