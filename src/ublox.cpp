#include "ublox/ublox.h"
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
using namespace ublox;

/////////////////////////////////////////////////////
// includes for default time callback
#define WIN32_LEAN_AND_MEAN
#define PI 3.14159265
#include "boost/date_time/posix_time/posix_time.hpp"
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

inline void DefaultErrorMsgCallback(const std::string &msg) {
    std::cout << "Ublox Error: " << msg << std::endl;
}

inline void DefaultNavPosLlhCallback(NavPosLLH nav_position, double time_stamp){
    std:: cout << "NAV-POSLLH: \n" <<
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

Ublox::Ublox() {
	serial_port_=NULL;
	reading_status_=false;
	time_handler_ = DefaultGetTime;
    handle_acknowledgement_=DefaultAcknowledgementHandler;
    nav_pos_llh_callback_=DefaultNavPosLlhCallback;
	aid_eph_callback_=DefaultAidEphCallback;
	aid_alm_callback_=DefaultAidAlmCallback;
    //nav_sol_callback_;
    //nav_vel_ned_callback;
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

        if (bytes_read<8)
            continue;

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


bool Ublox::RequestLogOnChanged(string logID) {

	cout << "RequestLogOnChanged " << endl;

	if(logID == "AIDEPH"){
	
		cout << "constructing Ephem Request message" << endl;
	
        unsigned char check[2];
		unsigned int length = 6;
		unsigned int totLength = 8;
	
        unsigned char data[8];
		// Header
		data[0] = 0xB5;
		data[1] = 0x62;
		// Message ID
		data[2] = 0x0B;
		data[3] = 0x31;
		// Length
		data[4] = 0;
		data[5] = 0;

		calculateCheckSum(data,length,check);

		// Check Sum
		data[6] = check[0];
		data[7] = check[1];

        return false;
        //return (SendString(data,totLength)==totLength);
	
	}
	else{

		return false;

	}
	return true;

}

//bool Ublox::PollMessage(uint8_t class_id, uint8_t msg_id){

//    CfgMsg message;
//    message.header.sync1=0xB5;
//    message.header.sync2=0x62;
//    message.header.message_class=0x06;
//    message.header.message_id=0x01;
//    message.header.payload_length=2;

//    message.message_class=class_id;
//    message.message_id=msg_id;

//    unsigned char* msg_ptr = (unsigned char*)&message;
//    calculateCheckSum(msg_ptr+2,6,message.checksum);

//    size_t bytes_written=serial_port_->write(msg_ptr, sizeof(message));
//    return bytes_written==sizeof(message);
//}

bool Ublox::PollMessage(uint8_t class_id, uint8_t msg_id){

    uint8_t message[8];

    message[0]=0xB5;    // sync 1
    message[1]=0x62;    // sync 2
    message[2]=class_id;
    message[3]=msg_id;
    message[4]=0;   // length 1
    message[5]=0;   // length 2
    message[6]=0;   // checksum 1
    message[7]=0;   // checksum 2

    uint8_t* msg_ptr = (uint8_t*)&message;
    calculateCheckSum(msg_ptr+2,4,msg_ptr+6);

    //printHex((char*)msg_ptr, 8);

    size_t bytes_written=serial_port_->write(message, 8);
    return bytes_written==8;
}

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

    //std::cout << "CfgMsg size: " << sizeof(message) << std::endl;

    unsigned char* msg_ptr = (unsigned char*)&message;
    calculateCheckSum(msg_ptr+2,7,message.checksum);

    serial_port_->write(msg_ptr, sizeof(message));
    return true;
}

void Ublox::SetPortConfiguration(bool ubx_input, bool ubx_output, bool nmea_input, bool nmea_output)
{
    CfgPrt message;
    std::cout << sizeof(message) << std::endl;
    message.header.sync1=0xB5;
    message.header.sync2=0x62;
    message.header.message_class=0x06;
    message.header.message_id=0x00;
    message.header.payload_length=20;

    message.port_id=3;
    message.reserved=0;
    message.reserved2=0;
    message.reserved3=0;
    message.reserved4=0;
    message.reserved5=0;

    message.tx_ready=0;

    message.input_mask=0;
    message.output_mask=0;

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

    serial_port_->write(msg_ptr, sizeof(message));
}



void Ublox::BufferIncomingData(uint8_t *msg, size_t length)
{
	//MOOSTrace("Inside BufferIncomingData\n");
    //cout << length << endl;
    //cout << 0 << ": " << dec << (int)msg[0] << endl;
	// add incoming data to buffer

    printHex(reinterpret_cast<char*>(msg),length);

	for (unsigned int i=0; i<length; i++)
	{
        //cout << i << ": " << hex << (int)msg[i] << dec << endl;
        // make sure buffer_index_ is not larger than buffer
        if (buffer_index_>=MAX_NOUT_SIZE)
		{
            buffer_index_=0;
            //MOOSTrace("ublox: Overflowed receive buffer. Reset");
		}

        //cout << "buffer_index_ = " << buffer_index_ << endl;
        //if (buffer_index_ > 200)
		//{
		//	exit(-1);
		//}
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
				readingACK=false;
			} // end if (msg[i]==0x62)
        }	// end else if (buffer_index_==1)
        else if (buffer_index_==2)
		{	//look for ack
			if (msg[i]==0x05)
			{
				//cout << "got ACK " << endl;
                buffer_index_ = 0;
                bytes_remaining_=0;
				readingACK = false;							//? Why is readingACK = false in if & else statement? - CC
			}
			else
			{
                data_buffer_[buffer_index_++]=msg[i];
				readingACK = false;							
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
			// length of message (payload + 2 byte check sum)
            bytes_remaining_ = ((data_buffer_[buffer_index_-1])<<8)+data_buffer_[buffer_index_-2]+1;
			
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
            //cout << " msgID = " << msgID << endl;
            ParseLog(data_buffer_,msgID);
			// reset counters
            buffer_index_=0;
            bytes_remaining_=0;
			//cout << "Message Done." << endl;
        }  // end else if (bytes_remaining_==1)
		else
		{	// add data to buffer
            data_buffer_[buffer_index_++]=msg[i];
            bytes_remaining_--;
		}
	}	// end for
}

bool Ublox::WaitForAck(int timeout) {
	// waits timeout # of seconds for an acknowledgement from the receiver
	// returns true if one is received, false for a timeout
	if (ackReceived)
		return true;
	else {
		for (int ii=0; ii<timeout; ii++){
			// sleep for 1 second
            //MOOSPause(1000);
			// check again
			if (ackReceived)
				return true;
		}
		return false;
	}
}

void Ublox::ParseLog(uint8_t *log, size_t logID)
{
	int length;

	switch (logID)
	{

        case NAV_POSLLH:
            NavPosLLH cur_nav_position;
            std::cout << sizeof(cur_nav_position) << std::endl;
            memcpy(&cur_nav_position, log, sizeof(cur_nav_position));
            /*std::cout << "header: " << std::endl;
            std::cout << hex << (int)cur_nav_position.header.message_class <<std::endl;
            std::cout << hex << (int)cur_nav_position.header.message_id <<std::endl;
            std::cout << dec << (int)cur_nav_position.header.payload_length <<std::endl;
            std::cout << dec << cur_nav_position.iTOW << std::endl;
            std::cout << dec << cur_nav_position.latitude_scaled << std::endl;*/

            nav_pos_llh_callback_(cur_nav_position, read_timestamp_);
            break;
		
		case AID_EPH:			
            EphemSV cur_eph_sv;
			gps_eph_data ret_eph_data;

			std::cout << "Ephem size: " << sizeof(cur_eph_sv) << std::endl;		// (112 bytes)
			memcpy(&cur_eph_sv, log, sizeof(cur_eph_sv));
			
			ret_eph_data = Ublox::Parse_aid_eph(cur_eph_sv);

			//Display Parsed Eph Data:
			cout << "PRN: " << ret_eph_data.prn << endl;
			cout << "T_GD: " << ret_eph_data.tgd << endl;
			cout << "t_oc: " << ret_eph_data.toc << endl;
			cout << "af0: " << ret_eph_data.af0 << endl;
			cout << "af1: " << ret_eph_data.af1 << endl;
			cout << "af2: " << ret_eph_data.af2 << endl;
			cout << "M_0: " << ret_eph_data.anrtime << endl;
			cout << "deltan: " << ret_eph_data.dN << endl;
			cout << "ecc: " << ret_eph_data.ecc << endl;
			cout << "sqrtA: " << ret_eph_data.majaxis << endl;
			cout << "OMEGA_0: " << ret_eph_data.wo << endl;
			cout << "i_0: " << ret_eph_data.ia << endl;
			cout << "Omega: " << ret_eph_data.omega << endl;
			cout << "Omega dot: " << ret_eph_data.dwo << endl;
			cout << "IDOT: " << ret_eph_data.dia << endl;
			cout << "C_uc: " << ret_eph_data.cuc << endl;
			cout << "C_us: " << ret_eph_data.cus << endl;
			cout << "C_rc: " << ret_eph_data.crc << endl;
			cout << "C_rs: " << ret_eph_data.crs << endl;
			cout << "C_is: " << ret_eph_data.cis << endl;
			cout << "t_oe: " << ret_eph_data.toe << endl;
			cout << "----------------------------------" << endl;
			cout << endl;

			aid_eph_callback_(cur_eph_sv, read_timestamp_);
			
				/*ubx.rxm_eph = (struct s_rxm_eph*) log;
				cout << "ubx length " << ubx.rxm_eph->UbloxHeader.payload_length << endl;
				cout << "ubx prn " << ubx.rxm_eph->svprn << endl;
				gpsephem_data Ublox::Parse_aid_eph(cur_eph_sv);
                 eph_data.prn=ubx.rxm_eph->svprn;
				PublishGpsEphemToDB(eph_data,MOOSTime());*/

			break;

		case AID_ALM:
			AlmSV cur_alm_sv;
			std::cout << "Alm size: " << sizeof(cur_alm_sv) << endl;			// 48 bytes
			memcpy(&cur_alm_sv, log, sizeof(cur_alm_sv));
			aid_alm_callback_(cur_alm_sv, read_timestamp_);

			break;

		case rangeID:
			
			//cout << "sizeoff range log Struct + " << sizeof(curRawData) << endl;
			//length = ((log[1])<<8) + log[0];
			//int numSv = log[8];
			//cout << "numSv from log =  " << numSv << endl;
			//cout << "length of Range message = " << length << endl;
            //memcpy(&curRawData, log+2, length);
			//PublishRangeToDB(curRawData);
			break;	
        case NAV_VELNED:

			//cout << "sizeoff NavSol Struct + " << sizeof(curNavSolData) << endl;
			//int length;
			//length = ((log[1])<<8)+log[0];
			//cout << "length of Nav message = " << length << endl;
            //memcpy(&curNavVelNed, log+2, sizeof(curNavVelNed));
			//PublishNavVelNedToDB(curNavVelNed);
			//cout << "Something Good NAVSOL" << endl;
			break;

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


gps_eph_data Ublox::Parse_aid_eph(EphemSV ubx_eph)
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
	
	gps_eph_data eph_data;

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



