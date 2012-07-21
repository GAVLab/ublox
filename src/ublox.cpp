#include "ublox/ublox.h"
#include <iostream>

using namespace std;
using namespace ublox;

/////////////////////////////////////////////////////
// includes for default time callback
//#define WIN32_LEAN_AND_MEAN
#define PI 3.14159265
//#include "boost/date_time/posix_time/posix_time.hpp"
////////////////////////////////////////////////////


inline void printHex(char *data, int length)
{
	for(int i = 0; i < length; ++i){
		printf("0x%.2X ", (unsigned)(unsigned char)data[i]);
	}
	printf("\n");
}

/*!
 * Default callback method for timestamping data.  Used if a
 * user callback is not set.  Returns the current time from the
 * CPU clock as the number of seconds from Jan 1, 1970
 */
double DefaultGetTime() {
	boost::posix_time::ptime present_time(boost::posix_time::microsec_clock::universal_time());
	boost::posix_time::time_duration duration(present_time.time_of_day());
	return duration.total_seconds();
}

void DefaultAcknowledgementHandler() {
    std::cout << "Acknowledgement received." << std::endl;
}

inline void DefaultDebugMsgCallback(const std::string &msg) {
    std::cout << "Ublox Debug: " << msg << std::endl;
}

inline void DefaultInfoMsgCallback(const std::string &msg) {
    std::cout << "Ublox Info: " << msg << std::endl;
}

inline void DefaultWarningMsgCallback(const std::string &msg) {
    std::cout << "Ublox Warning: " << msg << std::endl;
}

inline void DefaultPortSettingsCallback(CfgPrt port_settings, double time_stamp) {
    std::cout << "CFG-PRT:"<< std::endl;
}

inline void DefaultErrorMsgCallback(const std::string &msg) {
    std::cout << "Ublox Error: " << msg << std::endl;
}

inline void DefaultNavSolCallback(NavSol nav_sol, double time_stamp){
    std::cout << "NAV-SOL: " << endl;
}

inline void DefaultNavStatusCallback(NavStatus nav_status, double time_stamp){
    std::cout << "GPS Fix Type: ";
    if (nav_status.fixtype == 0x00){
        std::cout << "No Fix" << std::endl;
        std::cout << "TTFF: " <<  " none ms" << std::endl;
        std::cout << "Milliseconds since Startup/Reset: " << nav_status.msss << std::endl;
    }
    else if (nav_status.fixtype == 0x01){
        std::cout << "Dead Reckoning Only"<< std::endl;
    }
    else if (nav_status.fixtype == 0x02){
        std::cout << "2D Fix"<< std::endl;
    }
    else if (nav_status.fixtype == 0x03){
        std::cout << "3D Fix"<< std::endl;
    }
    else if (nav_status.fixtype == 0x04){
        std::cout << "GPS + Dead Reckoning"<< std::endl;
    }
    else if (nav_status.fixtype == 0x05){
        std::cout << "Time Only"<< std::endl;
    }
    else {
        std::cout << std::endl;
    }

    if (nav_status.fixtype != 0x00){
        std::cout << "TTFF: " << (nav_status.ttff/1000.) << " sec" << std::endl;
        std::cout << "Milliseconds since Startup/Reset: " << (nav_status.msss/1000.) << " sec" << std::endl;
    }
}

inline void DefaultNavVelNedCallback(NavVelNed nav_vel_ned, double time_stamp){
    std::cout << "NAV-VELNED: " << endl;
}

inline void DefaultNavPosLlhCallback(NavPosLLH nav_position, double time_stamp){
    std:: cout << "NAV-POSLLH: " << endl <<
                  "  GPS milliseconds: " << nav_position.iTOW << std::endl <<
                  "  Latitude: " << nav_position.latitude_scaled << std::endl <<
                  "  Longitude: " << nav_position.longitude_scaled << std::endl <<
                  "  Height: " << nav_position.height << std::endl << std::endl;
}

inline void DefaultAidEphCallback(EphemSV eph_sv, double time_stamp){
	std::cout << "AID-EPH: " << std::endl;
}

inline void DefaultAidAlmCallback(AlmSV alm_sv, double time_stamp){
	std::cout << "AID-ALM: " << std::endl;

}

inline void DefaultAidHuiCallback(AidHui aid_hui, double time_stamp){
    std::cout << "AID-HUI: " << std::endl;
}

inline void DefaultAidIniCallback(AidIni aid_ini, double time_stamp){
    std::cout << "AID-INI: " << std::endl;
}

inline void DefaultRxmRawCallback(RawMeas raw_meas, double time_stamp){
    std::cout << "RXM-RAW: " << std::endl;
}

inline void DefaultRxmSvsiCallback(SVStat sv_stat, double time_stamp){
    std::cout << "RXM-SVSI: " << std::endl;
}

Ublox::Ublox() {
	serial_port_=NULL;
	reading_status_=false;
	time_handler_ = DefaultGetTime;
    handle_acknowledgement_=DefaultAcknowledgementHandler;
    port_settings_callback_=DefaultPortSettingsCallback;
    nav_pos_llh_callback_=DefaultNavPosLlhCallback;
	aid_eph_callback_=DefaultAidEphCallback;
	aid_alm_callback_=DefaultAidAlmCallback;
    aid_hui_callback_=DefaultAidHuiCallback;
    aid_ini_callback_=DefaultAidIniCallback;
    rxm_raw_callback_=DefaultRxmRawCallback;
    rxm_svsi_callback_=DefaultRxmSvsiCallback;
    nav_sol_callback_=DefaultNavSolCallback;
    nav_status_callback_=DefaultNavStatusCallback;
    nav_vel_ned_callback_=DefaultNavVelNedCallback;
    log_debug_=DefaultDebugMsgCallback;
    log_info_=DefaultInfoMsgCallback;
    log_warning_=DefaultWarningMsgCallback;
    log_error_=DefaultErrorMsgCallback;
	reading_acknowledgement_=false;
    buffer_index_=0;
    read_timestamp_=0;
    parse_timestamp_=0;
}

Ublox::~Ublox() {
    Disconnect();
}

bool Ublox::Connect(std::string port, int baudrate) {
    //serial_port_ = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(1000));
    serial::Timeout my_timeout(100,1000,0,1000,0);
    serial_port_ = new serial::Serial(port,baudrate,my_timeout);

    if (!serial_port_->isOpen()){
        std::stringstream output;
        output << "Serial port: " << port << " failed to open.";
        log_error_(output.str());
        delete serial_port_;
        serial_port_ = NULL;
        return false;
    } else {
        std::stringstream output;
        output << "Serial port: " << port << " opened successfully.";
        log_info_(output.str());
    }

    //std::cout << "Flushing port" << std::endl;
    serial_port_->flush();

    // stop any incoming data and flush buffers
    // stop any incoming nmea data
    //SetPortConfiguration(true,true,false,false);
    // wait for data to stop cominig in
   // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
   // unsigned char result[5000];
   // size_t bytes_read;
   // bytes_read=serial_port_->read(result, 5000);
//    std::cout << result << std::endl;
//    std::cout << "flushing port" << std::endl;
//    // clear serial port buffers
//    serial_port_->flush();

    // turn off NMEA messages
   // ConfigureMessageRate(0x0F, 0x00, 0);
  //  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
   // bytes_read=serial_port_->read(result, 5000);
   // std::cout << result << std::endl;
   // std::cout << "flushing port" << std::endl;
    // clear serial port buffers
    //serial_port_->flush();

    // look for GPS by sending ping and waiting for response
    if (!Ping()){
        std::stringstream output;
        output << "Ublox GPS not found on port: " << port << std::endl;
        log_error_(output.str());
        delete serial_port_;
        serial_port_ = NULL;
        return false;
    }

    // start reading
    StartReading();
    return true;

}

bool Ublox::Ping(int num_attempts) {
    while ((num_attempts--)>0) {
        log_info_("Searching for Ublox receiver...");
        // request version information

        // ask for version
        PollMessage(0x0A,0x04);

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        unsigned char result[5000];
        size_t bytes_read;
        bytes_read=serial_port_->read(result, 5000);

        //std::cout << "bytes read: " << (int)bytes_read << std::endl;
        //std::cout << dec << result << std::endl;

        if (bytes_read<8) {
            std::cout << "Only read " << bytes_read << " bytes in response to ping." << std::endl;
            continue;
        }

        uint16_t length;
        // search through result for version message
        for (int ii=0; ii<(bytes_read-8); ii++) {
            //std::cout << hex << (unsigned int)result[ii] << std::endl;
            if (result[ii]==0xB5) {
                if (result[ii+1]!=0x62)
                    continue;
                if (result[ii+2]!=0x0A)
                    continue;
                if (result[ii+3]!=0x04)
                    continue;
                //std::cout << "length1:" << hex << (unsigned int)result[ii+4] << std::endl;
                //std::cout << "length2:" << hex << (unsigned int)result[ii+5] << std::endl;
                length=(result[ii+4])+(result[ii+5]<<8);
                if (length<40) {
                    log_warning_("Incomplete version message received");
                //    //return false;
                    continue;
                }

                string sw_version;
                string hw_version;
                string rom_version;
                sw_version.append((char*)(result+6));
                hw_version.append((char*)(result+36));
                //rom_version.append((char*)(result+46));
                log_info_("Ublox receiver found.");
                log_info_("Software Version: " + sw_version);
                log_info_("Hardware Version: " + hw_version);
                //log_info_("ROM Version: " + rom_version);
                return true;
            }
        }
        std::cout << "Read " << bytes_read << " bytes, but version message not found." << std::endl;

    }
    return false;
}

void Ublox::Disconnect() {
    StopReading();
    serial_port_->close();
    delete serial_port_;
    serial_port_=NULL;
}

void Ublox::StartReading() {
    // create thread to read from sensor
    reading_status_=true;
    read_thread_ptr_ = boost::shared_ptr<boost::thread >
        (new boost::thread(boost::bind(&Ublox::ReadSerialPort, this)));
}

void Ublox::StopReading() {
    reading_status_=false;
}

void Ublox::ReadSerialPort() {
    uint8_t buffer[MAX_NOUT_SIZE];
    size_t len;

    // continuously read data from serial port
    while (reading_status_) {
        // read data
        len = serial_port_->read(buffer, MAX_NOUT_SIZE);
        // timestamp the read
        read_timestamp_ = time_handler_();
        // add data to the buffer to be parsed
        BufferIncomingData(buffer, len);
    }

}

//////////////////////////////////////////////////////////////////////////////
// AIDING DATA POLL MESSAGES
/////////////////////////////////////////////////////////////////////////////

// Poll Message used to request for all SV
bool Ublox::PollMessage(uint8_t class_id, uint8_t msg_id){
    uint8_t message[8];

    message[0]=0xB5;        // sync 1
    message[1]=0x62;        // sync 2
    message[2]=class_id;
    message[3]=msg_id;
    message[4]=0;           // length 1
    message[5]=0;           // length 2
    message[6]=0;    // checksum 1
    message[7]=0;    // checksum 2

    uint8_t* msg_ptr = (uint8_t*)&message;

    calculateCheckSum(msg_ptr+2,4,msg_ptr+6);

    size_t bytes_written=serial_port_->write(message, 8);

    return bytes_written==8;
}

// Poll Message used to request for one SV
bool Ublox::PollMessageIndSV(uint8_t class_id, uint8_t msg_id, uint8_t svid){
    uint8_t message[9];

    message[0]=0xB5;        // sync 1
    message[1]=0x62;        // sync 2
    message[2]=class_id;
    message[3]=msg_id;
    message[4]=1;           // length 1
    message[5]=0;           // length 2
    message[6]=svid;        // Payload
    message[7]=0;           // checksum 1
    message[8]=0;           // checksum 2


    uint8_t* msg_ptr = (uint8_t*)&message;
    calculateCheckSum(msg_ptr+2,5,msg_ptr+7);
    size_t bytes_written=serial_port_->write(msg_ptr, 9);

    return bytes_written==9;
}

// (AID-EPH) Polls for Ephemeris data
bool Ublox::PollEphem(int8_t svid){

    if (svid < -1){
        std::cout << "Error in PollEphem: Invalid input 'svid'" << endl;
        return 0;
    }
    else if (svid == -1){ // Requests Ephemerides for all SVs
        std::cout << "Polling for all Ephemerides.." << endl;
        return PollMessage(0x0B, 0x31);
    }
    else if (svid > 0){ // Requests Ephemeris for a single SV
        std::cout << "Polling for SV# " << (int)svid << " Ephemeris.." << endl;
        return PollMessageIndSV(0x0B, 0x31, (uint8_t) svid);
    }
    else{
        std::cout << "Error in PollEphem: Invalid input 'svid'" << endl;
        return 0;
    }
}

// (AID-ALM) Polls for Almanac Data
bool Ublox::PollAlmanac(int8_t svid){

    if (svid < -1){
        std::cout << "Error in PollAlmanac: Invalid input 'svid'" << endl;
        return 0;
    }
    else if (svid == -1){ // Requests Almanac Data for all SVs
        std::cout << "Polling for all Almanac Data.." << endl;
        return PollMessage(0x0B, 0x30);
    }
    else if (svid > 0){ // Requests Almanac Data for a single SV
        std::cout << "Polling for SV# " << (int)svid << " Almanac Data.." << endl;
        return PollMessageIndSV(0x0B, 0x30, (uint8_t) svid);
    }
    else{
        std::cout << "Error in PollAlmanac: Invalid input 'svid'" << endl;
        return 0;
    }
}

// (AID-HUI) Polls GPS Health, UTC and Ionospheric Parameters
bool Ublox::PollHUI(){
    std::cout << "Polling for AID-HUI.." << endl;
    return PollMessage(0x0B, 0x02);
}

// (AID-INI) Polls for Receiver Position, Time, Frequency, and Clock Drift
bool Ublox::PollIniAid(){
    std::cout << "Polling for AID-INI.." << endl;
    return PollMessage(0x0B, 0x01);
}

// (AID-DATA) Polls for All AID Data (-INI, -HUI, -EPH, -ALM)
bool Ublox::PollAllAidData(){
    std::cout << "Polling for AID-HUI, AID-INI, AID-EPH, & AID-ALM.." << endl;
    return PollMessage(0x0B, 0x10);
}

// (RXM-RAW) Polls for Raw DGPS data
bool Ublox::PollRawDgpsData(){
    std::cout << "Polling for RXM-RAW.." << endl;
    return PollMessage(0x02, 0x10);
}

// (RXM-SVSI) Polls for Satellite Status Info
bool Ublox::PollSVStatus(){
    std::cout << "Polling for RXM-SVSI.." << endl;
    return PollMessage(0x02, 0x20);
}

// (NAV-STATUS) Polls for Receiver Navigation Status
bool Ublox::PollNavStatus(){
    std::cout << "Polling for Receiver NAV-STATUS.." << endl;
    return PollMessage(0x01, 0x03);
}

////////////////////////////////////////////////////////
// (CFG) Configuration Messages
////////////////////////////////////////////////////////
// Receiver Reset
bool Ublox::Reset(uint16_t nav_bbr_mask, uint8_t reset_mode){
    CfgRst message;

    message.header.sync1=0xB5;
    message.header.sync2=0x62;
    message.header.message_class=0x06;
    message.header.message_id=0x04;
    message.header.payload_length=4;

    message.nav_bbr_mask = nav_bbr_mask;    //X2-Bitfield?
        // Startup Modes
            // Hotstart 0x000
            // Warmstart 0x0001
            // Coldstart 0xFFFF
    message.reset_mode = reset_mode;
        // Reset Modes:
            // Hardware Reset 0x00
            // Controlled Software Reset 0x01
            // Controlled Software Reset - Only GPS 0x02
            // Hardware Reset After Shutdown 0x04
            // Controlled GPS Stop 0x08
            // Controlled GPS Start 0x09

    //message.reserved = 0;

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr+2,8,message.checksum);

    serial_port_->write(msg_ptr, 12);
    return true;
}

// Receiver Reset Messages - Force Cold Start
bool Ublox::ResetToColdStart(uint8_t reset_mode){
    std::cout << "Cold Start." << endl;
    return Reset(0xFFFF, reset_mode);
}

// Receiver Reset Messages - Force Warm Start
bool Ublox::ResetToWarmStart(){
    std::cout << "Warm Start." << endl;
    return Reset(0x0001, 0x02);
}

// Receiver Reset Messages - Force Hot Start
bool Ublox::ResetToHotStart(){
    std::cout << "Hot Start." << endl;
    return Reset(0x0000, 0x02);
}

// (CFG-MSG) Set message output rate for specified message
bool Ublox::ConfigureMessageRate(uint8_t class_id, uint8_t msg_id, uint8_t rate) {
	
    CfgMsgRate message;
    message.header.sync1=0xB5;
    message.header.sync2=0x62;
    message.header.message_class=0x06;
    message.header.message_id=0x01;
    message.header.payload_length=3;

    message.message_class=class_id;
    message.message_id=msg_id;
    message.rate=rate;

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr+2,7,message.checksum);

    serial_port_->write(msg_ptr, sizeof(message));
    return true;
}
// Set Port Configuration
 //NO ACK RETURNS - Prob
void Ublox::SetPortConfiguration(bool ubx_input, bool ubx_output, bool nmea_input, bool nmea_output)
{
    CfgPrt message;
    //std::cout << sizeof(message) << std::endl;
    message.header.sync1=0xB5;
    message.header.sync2=0x62;
    message.header.message_class=0x06;
    message.header.message_id=0x00;
    message.header.payload_length=20;

    message.port_id=3;          //Port identifier for USB Port (3)
    message.reserved=0;
    message.tx_ready=0;
    message.reserved2=0;
    message.reserved3=0;
    message.input_mask=0;       // Specifies input protocols
    message.output_mask=0;      // Specifies output protocols
    message.reserved4=0;
    message.reserved5=0;

    if (ubx_input)
        message.input_mask=message.input_mask | 0x0001;   // set first bit
    else
        message.input_mask=message.input_mask & 0xFFFE;   // clear first bit

    if (nmea_input)
        message.input_mask=message.input_mask | 0x0002;   // set second bit
    else
        message.input_mask=message.input_mask & 0xFFFD;   // clear second bit

    if (ubx_output)
        message.output_mask=message.output_mask | 0x0001;   // set first bit
    else
        message.output_mask=message.output_mask & 0xFFFE;   // clear first bit

    if (nmea_output)
        message.output_mask=message.output_mask | 0x0002;   // set second bit
    else
        message.output_mask=message.output_mask & 0xFFFD;   // clear second bit

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr+2,27,message.checksum);

    std::cout << "Set Port Settings Message Sent" << std::endl;
    printHex((char*) &message, sizeof(message));

    serial_port_->write(msg_ptr, sizeof(message));
    return;
}

// Poll Port Configuration
void Ublox::PollPortConfiguration(uint8_t port_identifier)
{                                   // Port identifier = 3 for USB (default value if left blank)
                                    //                 = 1 or 2 for UART
    uint8_t message[9];
    //std::cout << sizeof(message) << std::endl;
    message[0]=0xB5;
    message[1]=0x62;
    message[2]=0x06;
    message[3]=0x00;
    message[4]=1;
    message[5]=0;
    message[6]=port_identifier;         //Port identifier for USB Port (3)
    message[7]=0;                       // Checksum A
    message[8]=0;                       // Checksum B

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr+2,5,msg_ptr+7);

    serial_port_->write(msg_ptr, sizeof(message));
    std::cout << "Polling for Port Protocol Configuration." << std::endl;
    return;
}

//////////////////////////////////////////////////////////////
// Save/Read data to file
//////////////////////////////////////////////////////////////

bool Ublox::SaveEphemerides()
{
    // Serializing struct to student.data
        ofstream output_file("Ephemerides.data", ios::binary);
        output_file.write((char*)&cur_ephemerides, sizeof(cur_ephemerides));
        output_file.close();
        return true;
}

Ephemerides Ublox::LoadEphemerides()
{
    ifstream input_file("Ephemerides.data", ios::binary);
    input_file.read((char*)&stored_ephems, sizeof(stored_ephems));
    return stored_ephems;
}

bool Ublox::SaveAlmanac()
{
    // Serializing struct to student.data
        ofstream output_file("Almanac.data", ios::binary);
        output_file.write((char*)&cur_almanac, sizeof(cur_almanac));
        output_file.close();
        return true;
}

Almanac Ublox::LoadAlmanac()
{
    ifstream input_file("Almanac.data", ios::binary);
    input_file.read((char*)&stored_almanac, sizeof(stored_almanac));
    return stored_almanac;
}

//////////////////////////////////////////////////////////////
// Functions to Transmit Aiding Data to Receiver
//////////////////////////////////////////////////////////////
// Send Message
bool Ublox::SendMessage(uint8_t* msg_ptr, size_t length)
{
    //std::cout << length << std::endl;
    //std::cout << "Message Pointer" << endl;
    //printHex((char*) msg_ptr, length);
    size_t bytes_written=serial_port_->write(msg_ptr, length);
    //std::cout << bytes_written << std::endl;
    return true;
}

// Send AID-INI to Receiver
bool Ublox::SendAidIni(AidIni ini)
{  
    //std::cout << "Sending AID-INI to receiver.." << endl;
    unsigned char* msg_ptr = (unsigned char*)&ini;
    // calculate checksum
    calculateCheckSum(msg_ptr+2,4+ini.header.payload_length,ini.checksum);
    return SendMessage(msg_ptr, sizeof(ini));
}

// Send AID-EPH to Receiver
bool Ublox::SendAidEphem(Ephemerides ephems)
{

    std::cout << "Sending Ephemerides for available SVs." << endl;

    for(uint8_t prn_index=1; prn_index<=32; prn_index++){
        if (ephems.ephemsv[prn_index].header.payload_length == 8){
            //std::cout << "No AID-EPH data for PRN # " << (int)prn_index << " .." << endl;
        }
        else{
            //std::cout << "Sending AID-EPH for PRN # " << (int) ephems.ephemsv[prn_index].svprn << " .." << endl;
            uint8_t* msg_ptr = (uint8_t*)&ephems.ephemsv[prn_index];

            SendMessage(msg_ptr, sizeof(ephems.ephemsv[prn_index]));

            //sleep(.1);
        }

    }
    return true;
}
// Send AID-ALM to Receiver
bool Ublox::SendAidAlm(Almanac almanac)
{
    for(uint8_t prn_index=1; prn_index<=32; prn_index++){
        if (almanac.almsv[prn_index].svprn == 0){
            std::cout << "No AID-ALM data for PRN # " << (int)prn_index << " .." << endl;

        }

        else{
            std::cout << "Sending AID-ALM for PRN # " << (int) almanac.almsv[prn_index].svprn << " .." << endl;
            unsigned char* msg_ptr = (unsigned char*)&almanac.almsv[prn_index];
            SendMessage(msg_ptr, sizeof(almanac.almsv[prn_index]));
        }
    }
    return true;
}

// Send AID-HUI to Receiver
bool Ublox::SendAidHui()
{
    std::cout << "Sending AID-HUI to receiver.." << endl;
    unsigned char* msg_ptr = (unsigned char*)&cur_aid_hui;
    return SendMessage(msg_ptr, sizeof(cur_aid_hui));
}
// Send RXM-RAW to Receiver
bool Ublox::SendRawMeas()
{
    std::cout << "Sending RXM-RAW to receiver.." << endl;
    unsigned char* msg_ptr = (unsigned char*)&cur_raw_meas;
    return SendMessage(msg_ptr, sizeof(cur_raw_meas));
}
//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
void Ublox::BufferIncomingData(uint8_t *msg, size_t length)
{
	//MOOSTrace("Inside BufferIncomingData\n");
    //cout << length << endl;
    //cout << 0 << ": " << dec << (int)msg[0] << endl;
	// add incoming data to buffer

    //printHex(reinterpret_cast<char*>(msg),length);

	for (unsigned int i=0; i<length; i++)
	{
        //cout << i << ": " << hex << (int)msg[i] << dec << endl;
        // make sure buffer_index_ is not larger than buffer
        if (buffer_index_>=MAX_NOUT_SIZE)
		{
            buffer_index_=0;
            std::cout << "Overflowed receiver buffer. See ublox.cpp BufferIncomingData" << endl;

		}
          //cout << "buffer_index_ = " << buffer_index_ << endl;

        if (buffer_index_==0)
		{	// looking for beginning of message
			if (msg[i]==0xB5)
			{	// beginning of msg found - add to buffer
                //cout << "got first bit" << endl;
                data_buffer_[buffer_index_++]=msg[i];
                bytes_remaining_=0;
			}	// end if (msg[i]
        } // end if (buffer_index_==0)
        else if (buffer_index_==1)
		{	// verify 2nd character of header
			if (msg[i]==0x62)
			{	// 2nd byte ok - add to buffer
                //cout << " got second synch bit" << endl;
                data_buffer_[buffer_index_++]=msg[i];
			}
			else
			{
				// start looking for new message again
                buffer_index_=0;
                bytes_remaining_=0;
                //readingACK=false;
			} // end if (msg[i]==0x62)
        }	// end else if (buffer_index_==1)
        else if (buffer_index_==2)
        {	//look for ack

            if (msg[i]==0x05)   // ACK or NAK message class
			{
                // Get message id from payload
                char* class_id = reinterpret_cast<char*>(msg[i+4]);
                char* msg_id = reinterpret_cast<char*>(msg[i+5]);

                // Add function which takes class_id and msg_id and returns name of corresponding message

                if (msg[i+1]==0x01) // ACK Message
                {
                   std::cout << "Receiver Acknowledged Message " << std::endl;
                    //printf("0x%.2X ", (unsigned)class_id);
                    //std::cout << " ";
                    //printf("0x%.2X ", (unsigned)msg_id);
                    //std::cout << endl;

                }

                else if (msg[i+1]==0x00)    // NAK Message
                {
                    std::cout << "Receiver Did Not Acknowledged Message " << std::endl;
                     //printf("0x%.2X ", (unsigned)class_id);
                     //std::cout << " ";
                     //printf("0x%.2X ", (unsigned)msg_id);
                     //std::cout << endl;
                }

                buffer_index_ = 0;
                bytes_remaining_=0;
                //readingACK = false;			//? Why is readingACK = false in if & else statement? - CC
            }
            else
            {
                data_buffer_[buffer_index_++]=msg[i];
                //readingACK = false;
            }
		}
        else if (buffer_index_==3)
		{	
			// msg[i] and msg[i-1] define message ID
            data_buffer_[buffer_index_++]=msg[i];
			// length of header is in byte 4
			
            //printHex(reinterpret_cast < char * > (data_buffer_),4);
			
			
            msgID=((data_buffer_[buffer_index_-2])<<8)+data_buffer_[buffer_index_-1];
            //cout << "msgID = " << msgID << endl;
		}
        else if (buffer_index_==5)
		{	
			// add byte to buffer
            data_buffer_[buffer_index_++]=msg[i];
            // length of message (payload + 2 byte check sum )
            bytes_remaining_ = ((data_buffer_[buffer_index_-1])<<8)+data_buffer_[buffer_index_-2]+2;
			
            //cout << "bytes_remaining_ = " << bytes_remaining_ << endl;
			
			///cout << msgID << endl;
		}
        else if (buffer_index_==6)
		{	// set number of bytes
            data_buffer_[buffer_index_++]=msg[i];
            bytes_remaining_--;
		}
        else if (bytes_remaining_==1)
		{	// add last byte and parse
            data_buffer_[buffer_index_++]=msg[i];
            //std::cout << hex << (int)msg[i] << dec << std::endl;
            //cout << " msgID = " << msgID << std::endl;
            ParseLog(data_buffer_,msgID);
			// reset counters
            buffer_index_=0;
            bytes_remaining_=0;
            //cout << "Message Done." << std::endl;
        }  // end else if (bytes_remaining_==1)
		else
		{	// add data to buffer
            data_buffer_[buffer_index_++]=msg[i];
            bytes_remaining_--;
		}
	}	// end for
}

void Ublox::ParseLog(uint8_t *log, size_t logID)
{
    double length;


	switch (logID)
    {

        case AID_REQ:
            std::cout << "AID-REQ message received by computer." << endl;

        case CFG_PRT:
            //CfgPrt cur_port_settings;

            memcpy(&cur_port_settings, log, sizeof(cur_port_settings));

            printHex((char*) &cur_port_settings, sizeof(cur_port_settings));

            port_settings_callback_(cur_port_settings, read_timestamp_);

        break;

        case NAV_STATUS:
            //NavStatus cur_nav_status;

            memcpy(&cur_nav_status, log, sizeof(cur_nav_status));

            if (nav_status_callback_)
                nav_status_callback_(cur_nav_status, read_timestamp_);
            break;

        case NAV_SOL:
            //NavSol cur_nav_sol;

            memcpy(&cur_nav_sol, log, sizeof(cur_nav_sol));

            if (nav_sol_callback_)
                nav_sol_callback_(cur_nav_sol, read_timestamp_);
            break;

        case NAV_VELNED:
            //NavVelNed cur_nav_vel_ned;

            memcpy(&cur_nav_vel_ned, log, sizeof(cur_nav_vel_ned));

            if (nav_vel_ned_callback_)
                nav_vel_ned_callback_(cur_nav_vel_ned, read_timestamp_);
            break;

        case NAV_POSLLH:
            //NavPosLLH cur_nav_position;
            //std::cout << sizeof(cur_nav_position) << std::endl;
            memcpy(&cur_nav_position, log, sizeof(cur_nav_position));

            /*std::cout << "header: " << std::endl;
            std::cout << hex << (int)cur_nav_position.header.message_class <<std::endl;
            std::cout << hex << (int)cur_nav_position.header.message_id <<std::endl;
            std::cout << dec << (int)cur_nav_position.header.payload_length <<std::endl;
            std::cout << dec << cur_nav_position.iTOW << std::endl;
            std::cout << dec << cur_nav_position.latitude_scaled << std::endl;*/

            if (nav_pos_llh_callback_)
                nav_pos_llh_callback_(cur_nav_position, read_timestamp_);
            break;
		
        case AID_EPH:
            EphemSV cur_ephem_sv;
            // determine length of log to see if it contains satellite
            // information or just the header
            length = (double) *(log+4);

            //std::cout << "log length: " << length << std::endl;

            // If Ephemeris for SV is not present (length is 8 bytes)
            if (length == 8){
                // nothing to parse - no ephemerides for this satellite
                //std::cout << "SV# " << (double) *(log+6) << "- no ephemeris" << std::endl;
                memcpy(&cur_ephem_sv, log, 8+sizeof(UbloxHeader));

                // make sure functino pointer is set and call callback
                if (aid_eph_callback_)
                    aid_eph_callback_(cur_ephem_sv, read_timestamp_);
            }
            // If Ephemeris for SV is present (length is 104 bytes)
            else if (length == 104){
                memcpy(&cur_ephem_sv, log, sizeof(cur_ephem_sv));

                //printHex((char*) &cur_ephem_sv, sizeof(cur_ephem_sv));

                // Store each SV's ephemeris message in a structure
                cur_ephemerides.ephemsv[cur_ephem_sv.svprn] = cur_ephem_sv;

                // make sure functino pointer is set and call callback
                if (aid_eph_callback_)
                    aid_eph_callback_(cur_ephem_sv, read_timestamp_);
            }

            else{
                cout << "Error! AID-EPH log payload is not a valid length! (See ParseLog case AID_EPH)" << std::endl;
            }

            /*
			//Display Parsed Eph Data:
            cout << "PRN: " << parsed_ephem_data.prn << std::endl;
            cout << "T_GD: " << parsed_ephem_data.tgd << std::endl;
            cout << "t_oc: " << parsed_ephem_data.toc << std::endl;
            cout << "af0: " << parsed_ephem_data.af0 << std::endl;
            cout << "af1: " << parsed_ephem_data.af1 << std::endl;
            cout << "af2: " << parsed_ephem_data.af2 << std::endl;
            cout << "M_0: " << parsed_ephem_data.anrtime << std::endl;
            cout << "deltan: " << parsed_ephem_data.dN << std::endl;
            cout << "ecc: " << parsed_ephem_data.ecc << std::endl;
            cout << "sqrtA: " << parsed_ephem_data.majaxis << std::endl;
            cout << "OMEGA_0: " << parsed_ephem_data.wo << std::endl;
            cout << "i_0: " << parsed_ephem_data.ia << std::endl;
            cout << "Omega: " << parsed_ephem_data.omega << std::endl;
            cout << "Omega dot: " << parsed_ephem_data.dwo << std::endl;
            cout << "IDOT: " << parsed_ephem_data.dia << std::endl;
            cout << "C_uc: " << parsed_ephem_data.cuc << std::endl;
            cout << "C_us: " << parsed_ephem_data.cus << std::endl;
            cout << "C_rc: " << parsed_ephem_data.crc << std::endl;
            cout << "C_rs: " << parsed_ephem_data.crs << std::endl;
            cout << "C_is: " << parsed_ephem_data.cis << std::endl;
            cout << "t_oe: " << parsed_ephem_data.toe << std::endl;
            cout << "----------------------------------" << std::endl;
            cout << std::endl;
            */


			break;

		case AID_ALM:
			AlmSV cur_alm_sv;

            length = (double) *(log+4);
            memset(&cur_alm_sv, 0, sizeof(cur_alm_sv));

            // If Almanac data for SV is not present (length is 8 bytes)
            if (length == 8){
                std::cout << "SV# " << (double) *(log+6) << "- no almanac data" << std::endl;
                break;
            }

            // If Almanac data for SV is present (length is 40 bytes)
            else if (length == 40){
                memcpy(&cur_alm_sv, log, sizeof(cur_alm_sv));

                // Store each SV's almanac message in a structure
                cur_almanac.almsv[cur_alm_sv.svprn] = cur_alm_sv;

                // Put function to parse almanac parameters here
                //
                // Store almanac data for all SVs in one structure
                //
            }

            else{
                cout << "Error! AID-ALM log payload is not 8 or 40 bytes long! (See ParseLog case AID_ALM)" << std::endl;
            }

			aid_alm_callback_(cur_alm_sv, read_timestamp_);

			break;

        //case rangeID:
            //cout << "sizeoff range log Struct + " << sizeof(curRawData) << std::endl;
			//length = ((log[1])<<8) + log[0];
			//int numSv = log[8];
            //cout << "numSv from log =  " << numSv << std::endl;
            //cout << "length of Range message = " << length << std::endl;
            //memcpy(&curRawData, log+2, length);
			//PublishRangeToDB(curRawData);
            //break;

        case AID_HUI:
            //AidHui cur_aid_hui;

            memcpy(&cur_aid_hui, log, sizeof(cur_aid_hui));

            aid_hui_callback_(cur_aid_hui, read_timestamp_);
            std::cout << "Received AID-HUI Message." << std::endl;
            break;

        case AID_INI:
            //AidIni cur_aid_ini;

            //std::cout << "Received AID-INI Message." << std::endl;
            memcpy(&cur_aid_ini, log, sizeof(cur_aid_ini));

            //std::cout << "LOG message:" << std::endl;
            //printHex((char*) log, sizeof(cur_aid_ini));
            //std::cout << "Message stored in cur_aid_ini:" << std::endl;
            //printHex((char*) &cur_aid_ini, sizeof(cur_aid_ini));

            if (aid_ini_callback_)
                aid_ini_callback_(cur_aid_ini, read_timestamp_);

            break;

        case RXM_RAW:
            //RawMeas cur_raw_meas;
            length = (double) *(log+4);         // length = 8+24*numSV
            std::cout << "Length of RXM-RAW " << length << std::endl;

            memcpy(&cur_raw_meas, log, length);
        /*
            std::cout << "num of SVs " << (double)cur_raw_meas.numSV << std::endl;
            std::cout << "GPS Week " << cur_raw_meas.week << std::endl;
            std::cout << "Carrier Phase " << cur_raw_meas.rawmeasreap[0].cpmeas << endl;
            std::cout << "SV ID " << (float)cur_raw_meas.rawmeasreap[0].svid << endl;
            std::cout << "SV ID " << (float)cur_raw_meas.rawmeasreap[1].svid << endl;
            std::cout << "SV ID " << (double)cur_raw_meas.rawmeasreap[2].svid << endl;
            std::cout << "SV ID " << (double)cur_raw_meas.rawmeasreap[3].svid << endl;
            std::cout << "SV ID " << (double)cur_raw_meas.rawmeasreap[4].svid << endl;
        */
            if (rxm_raw_callback_)
                rxm_raw_callback_(cur_raw_meas, read_timestamp_);

            break;
/*
         case RXM_SVSI:
            //SVStat cur_sv_stat;

            length = (double) *(log+4);
            std::cout << "Length of RXM-SVSI " << length << endl;

            memcpy(&cur_sv_stat, log, length);

            rxm_svsi_callback_(cur_sv_stat, read_timestamp_);

            break;
*/
    }
}


void Ublox::calculateCheckSum(uint8_t* in, size_t length, uint8_t* out)
{

    uint8_t a = 0;
    uint8_t b = 0;
		
    for(uint8_t i=0; i<length; i++)
	{
		
		a = a + in[i];
		b = b + a;

	}
	
	out[0] = (a & 0xFF);
	out[1] = (b & 0xFF);

}


ParsedEphemData Ublox::Parse_aid_eph(EphemSV ubx_eph)
{
	union {
		unsigned short s;
		unsigned char c[2];
	} union_unsigned_short;
	union {
		short s;
		unsigned char c[2];
	} union_short;
	union {
		unsigned int i;
		unsigned char c[4];
	} union_unsigned_int;
	union {
		int i;
		unsigned char c[4];
	} union_int;
	
    ParsedEphemData eph_data;

	//SVID
	eph_data.prn = (uint8_t) ubx_eph.svprn;

	//T_GD
	eph_data.tgd = ((char) ubx_eph.SF[0].W[4].byte[0]) * pow(2.0,-31);

	//t_oc
	union_unsigned_short.c[0] = ubx_eph.SF[0].W[5].byte[0];
	union_unsigned_short.c[1] = ubx_eph.SF[0].W[5].byte[1];
	eph_data.toc = ((double) union_unsigned_short.s) * pow(2.0,4);
	
	//a_f2
	eph_data.af2 = ((char) ubx_eph.SF[0].W[6].byte[2]) * pow(2.0,-55);

	//a_f1
	union_short.c[0] = ubx_eph.SF[0].W[6].byte[0];
	union_short.c[1] = ubx_eph.SF[0].W[6].byte[1];
	eph_data.af1 = ((double) union_short.s) * pow(2.0,-43);
	 
	 //a_f0
	 union_int.c[0] = ubx_eph.SF[0].W[7].byte[0];
	 union_int.c[1] = ubx_eph.SF[0].W[7].byte[1];
	 union_int.c[2] = ubx_eph.SF[0].W[7].byte[2];
	 union_int.i = union_int.i>>2;
	
	 if ((union_int.c[2]>>5)&0x01)
	 {
	 union_int.c[2] = union_int.c[2] | 0xC0;
	 union_int.c[3] = 0xff;
	 }
	 else
	 {
	 union_int.c[2] = union_int.c[2] & 0x3F;
	 union_int.c[3] = 0x00;
	 }
	 eph_data.af0 = ((double) union_int.i) * pow(2.0,-31);
	 
	 //M_0 - Mean Anomoly Reference Time
	 union_int.c[0] = ubx_eph.SF[1].W[2].byte[0];
	 union_int.c[1] = ubx_eph.SF[1].W[2].byte[1];
	 union_int.c[2] = ubx_eph.SF[1].W[2].byte[2];
	 union_int.c[3] = ubx_eph.SF[1].W[1].byte[0];
	 eph_data.anrtime = ((double) union_int.i) * pow(2.0,-31) * PI;
	 
	 //deltan
	 union_short.c[0] = ubx_eph.SF[1].W[1].byte[1];
	 union_short.c[1] = ubx_eph.SF[1].W[1].byte[2];
	 eph_data.dN = ((double) union_short.s) * pow(2.0,-43) * PI;
	 
	 //ecc - Eccentricity
	 union_unsigned_int.c[0] = ubx_eph.SF[1].W[4].byte[0];
	 union_unsigned_int.c[1] = ubx_eph.SF[1].W[4].byte[1];
	 union_unsigned_int.c[2] = ubx_eph.SF[1].W[4].byte[2];
	 union_unsigned_int.c[3] = ubx_eph.SF[1].W[3].byte[0];
	 eph_data.ecc = ((double) union_unsigned_int.i) * pow(2.0,-33);
	 
	 //sqrtA - Square root of the semi-major axis
	 union_unsigned_int.c[0] = ubx_eph.SF[1].W[6].byte[0];
	 union_unsigned_int.c[1] = ubx_eph.SF[1].W[6].byte[1];
	 union_unsigned_int.c[2] = ubx_eph.SF[1].W[6].byte[2];
	 union_unsigned_int.c[3] = ubx_eph.SF[1].W[5].byte[0];
	 eph_data.majaxis = ((double) union_unsigned_int.i) * pow(2.0,-19);
	 
	 //OMEGA_0 - Longitude of Ascending Node of Orbit Plane at Weekly Epoch
	 union_int.c[0] = ubx_eph.SF[2].W[1].byte[0];
	 union_int.c[1] = ubx_eph.SF[2].W[1].byte[1];
	 union_int.c[2] = ubx_eph.SF[2].W[1].byte[2];
	 union_int.c[3] = ubx_eph.SF[2].W[0].byte[0];
	 eph_data.wo = ((double) union_int.i) * pow(2.0,-31) * PI;
	 
	 //i_0 - Inclination Angle
	 union_int.c[0] = ubx_eph.SF[2].W[3].byte[0];
	 union_int.c[1] = ubx_eph.SF[2].W[3].byte[1];
	 union_int.c[2] = ubx_eph.SF[2].W[3].byte[2];
	 union_int.c[3] = ubx_eph.SF[2].W[2].byte[0];
	 eph_data.ia = ((double) union_int.i) * pow(2.0,-31) * PI;
	 
	 //omega
	 union_int.c[0] = ubx_eph.SF[2].W[5].byte[0];
	 union_int.c[1] = ubx_eph.SF[2].W[5].byte[1];
	 union_int.c[2] = ubx_eph.SF[2].W[5].byte[2];
	 union_int.c[3] = ubx_eph.SF[2].W[4].byte[0];
	 eph_data.omega = ((double) union_int.i) * pow(2.0,-31) * PI;
	 
	 //OMEGADOT
	 union_int.c[0] = ubx_eph.SF[2].W[6].byte[0];
	 union_int.c[1] = ubx_eph.SF[2].W[6].byte[1];
	 union_int.c[2] = ubx_eph.SF[2].W[6].byte[2];
	 
	 if ((union_int.c[2]>>7)&0x01)
	 {
	 union_int.c[3] = 0xff;
	 }
	 else
	 {
	 union_int.c[3] = 0x00;
	 }
	 eph_data.dwo = ((double) union_int.i) * pow(2.0,-43) * PI;
	 
	 //IDOT - Rate of Inclination Angle
	 union_short.c[0] = ubx_eph.SF[2].W[7].byte[0];
	 union_short.c[1] = ubx_eph.SF[2].W[7].byte[1];
	 union_short.s = union_short.s>>2;
	 if ((union_short.c[1]>>5)&0x01)
	 {
	 union_int.c[1] = union_short.c[1] | 0xC0;
	 }
	 else
	 {
	 union_int.c[1] = union_short.c[1] & 0x3F;
	 }
	 eph_data.dia = ((double) union_short.s) * pow(2.0,-43) * PI;
	 
	 //C_uc
	 union_short.c[0] = ubx_eph.SF[1].W[3].byte[1];
	 union_short.c[1] = ubx_eph.SF[1].W[3].byte[2];
	 eph_data.cuc = ((double) union_short.s) * pow(2.0,-29);
	 
	 //C_us
	 union_short.c[0] = ubx_eph.SF[1].W[5].byte[1];
	 union_short.c[1] = ubx_eph.SF[1].W[5].byte[2];
	 eph_data.cus = ((double) union_short.s) * pow(2.0,-29);
	 
	 //C_rc
	 union_short.c[0] = ubx_eph.SF[2].W[4].byte[1];
	 union_short.c[1] = ubx_eph.SF[2].W[4].byte[2];
	 eph_data.crc = ((double) union_short.s) * pow(2.0,-5);
	 
	 //C_rs
	 union_short.c[0] = ubx_eph.SF[1].W[0].byte[0];
	 union_short.c[1] = ubx_eph.SF[1].W[0].byte[1];
	 eph_data.crs = ((double) union_short.s) * pow(2.0,-5);
	 
	 //C_ic
	 union_short.c[0] = ubx_eph.SF[2].W[0].byte[1];
	 union_short.c[1] = ubx_eph.SF[2].W[0].byte[2];
	 eph_data.cic = ((double) union_short.s) * pow(2.0,-29);
	 
	 //C_is
	 union_short.c[0] = ubx_eph.SF[2].W[2].byte[1];
	 union_short.c[1] = ubx_eph.SF[2].W[2].byte[2];
	 eph_data.cis = ((double) union_short.s) * pow(2.0,-29);
	 
	 //t_oe
	 union_unsigned_short.c[0] = ubx_eph.SF[1].W[7].byte[1];
	 union_unsigned_short.c[1] = ubx_eph.SF[1].W[7].byte[2];
	 eph_data.toe = ((double) union_unsigned_short.s) * pow(2.0,4);

	 return (eph_data);
};










