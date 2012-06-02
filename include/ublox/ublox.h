#ifndef UBLOX_H
#define UBLOX_H

#include "ublox_structures.h"
#include <serial/serial.h>

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace ublox {

typedef boost::function<double()> GetTimeCallback;
typedef boost::function<void()> HandleAcknowledgementCallback;

// Messaging callbacks
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarningMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;

// GPS Data Callbacks
typedef boost::function<void(NavPosLLH&, double&)> NavPosLLHCallback;
typedef boost::function<void(NavSol&, double&)> NavSolCallback;
typedef boost::function<void(NavVelNed&, double&)> NavVelNedCallback;

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

    bool ConfigureMessageRate(uint8_t class_id, uint8_t msg_id, uint8_t rate);
    bool RequestLogOnChanged(std::string log); //!< request the given log from the receiver at the given rate
	bool WaitForAck(int timeout); //!< waits for an ack from receiver (timeout in seconds)

	void BufferIncomingData(unsigned char* msg, unsigned int length);
	//! Function to parse logs into a usable structure
	void ParseLog(unsigned char* log, unsigned int logID);
	void Parse_rxm_eph();

    //////////////////////////////////////////////////////
    // Serial port reading members
    //////////////////////////////////////////////////////
	//! Serial port object for communicating with sensor
	serial::Serial *serial_port_;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;  //!< True if the read thread is running, false otherwise.

	//////////////////////////////////////////////////////
    // Diagnostic Callbacks
    //////////////////////////////////////////////////////
    HandleAcknowledgementCallback handle_acknowledgement_;
    DebugMsgCallback log_debug_;
    InfoMsgCallback log_info_;
    WarningMsgCallback log_warning_;
    ErrorMsgCallback log_error_;

    GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping


    //////////////////////////////////////////////////////
    // New Data Callbacks
    //////////////////////////////////////////////////////
    NavPosLLHCallback nav_pos_llh_callback_;
	NavSolCallback nav_sol_callback_;
	NavVelNedCallback nav_vel_ned_callback;

	bool ackReceived;
	bool readingACK;
	
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
	int hdrLength;
	unsigned short msgID;
	struct s_ubx ubx;
	
    void calculateCheckSum(unsigned char* in, unsigned int length, unsigned char* out);

};
}
#endif
