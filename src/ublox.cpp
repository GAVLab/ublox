#include "ublox/ublox.h"
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
using namespace ublox;

/////////////////////////////////////////////////////
// includes for default time callback
#define WIN32_LEAN_AND_MEAN
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
    std:: cout << "NAV-POSLLH: \n" << std::endl; /*GPS Week: " << best_position.header.gps_week <<
                  "  GPS milliseconds: " << best_position.header.gps_millisecs << std::endl <<
                  "  Latitude: " << best_position.latitude << std::endl <<
                  "  Longitude: " << best_position.longitude << std::endl <<
                  "  Height: " << best_position.height << std::endl << std::endl;*/
}

Ublox::Ublox() {
	serial_port_=NULL;
	reading_status_=false;
	time_handler_ = DefaultGetTime;
    handle_acknowledgement_=DefaultAcknowledgementHandler;
   	NavPosLLHCallback nav_pos_llh_callback_;
	NavSolCallback nav_sol_callback_;
	NavVelNedCallback nav_vel_ned_callback;
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
    serial::Timeout my_timeout(1000,50,0,50,0);
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

    std::cout << "Flushing port" << std::endl;
    serial_port_->flush();

    // stop any incoming data and flush buffers
    // stop any incoming nmea data
    //SetPortConfiguration(true,true,false,false);
    // wait for data to stop cominig in
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    unsigned char result[5000];
    size_t bytes_read;
    bytes_read=serial_port_->read(result, 5000);
    std::cout << result << std::endl;
    std::cout << "flushing port" << std::endl;
    // clear serial port buffers
    serial_port_->flush();

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
        unsigned char poll_request[8];
        poll_request[0]=0xB5;
        poll_request[1]=0x62;
        poll_request[2]=0x0A;
        poll_request[3]=0x04;
        poll_request[4]=0x00;
        poll_request[5]=0x00;
        calculateCheckSum(poll_request+2, 4, poll_request+6);

        //serial_port_->write(poll_request, 8);

        // wait for response
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        unsigned char result[5000];
        size_t bytes_read;
        bytes_read=serial_port_->read(result, 5000);

        std::cout << "bytes read: " << bytes_read << std::endl;
        std::cout << result << std::endl;

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
                std::cout << "length1:" << hex << (unsigned int)result[ii+4] << std::endl;
                std::cout << "length2:" << hex << (unsigned int)result[ii+5] << std::endl;
                length=(result[ii+4])+(result[ii+5]<<8);
                if (length<70) {
                    log_warning_("Incomplete version message received");
                    //return false;
                    continue;
                }

                string sw_version;
                string hw_version;
                string rom_version;
                sw_version.append((char*)(result+6));
                hw_version.append((char*)(result+36));
                rom_version.append((char*)(result+46));
                log_info_("Ublox receiver found.");
                log_info_("Software Version: " + sw_version);
                log_info_("Hardware Version: " + hw_version);
                log_info_("ROM Version: " + rom_version);
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
    unsigned char buffer[MAX_NOUT_SIZE];
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

	if(logID == "RXMEPH"){
	
		cout << "constructing Ephem Request message" << endl;
	
        unsigned char check[2];
		unsigned int length = 6;
		unsigned int totLength = 8;
	
        unsigned char data[8];
		// Header
		data[0] = 0xB5;
		data[1] = 0x62;
		// Message ID
		data[2] = 0x02;
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

bool Ublox::ConfigureMessageRate(uint8_t class_id, uint8_t msg_id, uint8_t rate) {
	
    CfgMsg message;
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



void Ublox::BufferIncomingData(unsigned char *msg, unsigned int length)
{
	//MOOSTrace("Inside BufferIncomingData\n");
	//cout << length << endl;
    //cout << 0 << ": " << dec << (int)msg[0] << endl;
	// add incoming data to buffer
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
				readingACK = false;
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
			//cout << " msgID = " << msgID << endl;
            ParseLog(data_buffer_+4,msgID);
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

void Ublox::ParseLog(unsigned char *log, unsigned int logID)
{

	int length;

	switch (logID)
	{

        case NAV_SOL:

            //memcpy(&curNavSolData, log+2, sizeof(curNavSolData));
			//PublishNavSolToDB(curNavSolData);
			break;
        case RXM_EPH:

			
			length = ((log[1])<<8) + log[0];
			if (length == 0x68)
			{
							
                //ubx.rxm_eph = (struct s_rxm_eph*) log;
			//	cout << "ubx length " << ubx.rxm_eph->len << endl;
			//	cout << "ubx prn " << ubx.rxm_eph->svprn << endl;
                //Parse_rxm_eph();
                //curGpsEphem.prn=ubx.rxm_eph->svprn;
			//	PublishGpsEphemToDB(curGpsEphem,MOOSTime());
				
			}
			
			break;
		case rangeID:
			
			//cout << "sizeoff range log Struct + " << sizeof(curRawData) << endl;
			
			length = ((log[1])<<8) + log[0];
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


void Ublox::calculateCheckSum(unsigned char* in, unsigned int length, unsigned char* out)
{

	unsigned int a = 0;
	unsigned int b = 0;
		
	for(unsigned int i=2; i<length; i++)
	{
		
		a = a + in[i];
		b = b + a;

	}
	
	out[0] = (a & 0xFF);
	out[1] = (b & 0xFF);


}


void Ublox::Parse_rxm_eph()
{
/*
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

	//cout << "size of ubx " << sizeof(ubx) << endl;
	
	//T_GD
	curGpsEphem.tgd = ((char) ubx.rxm_eph->SF[0].W[4].bit[0]) * pow(2.0,-31);
	//t_oc
	union_unsigned_short.c[0] = ubx.rxm_eph->SF[0].W[5].bit[0];
	union_unsigned_short.c[1] = ubx.rxm_eph->SF[0].W[5].bit[1];
	curGpsEphem.toc = ((double) union_unsigned_short.s) * pow(2.0,4);
	
	//a_f2
	curGpsEphem.af2 = ((char) ubx.rxm_eph->SF[0].W[6].bit[2]) * pow(2.0,-55);
	 
	//a_f1
	union_short.c[0] = ubx.rxm_eph->SF[0].W[6].bit[0];
	union_short.c[1] = ubx.rxm_eph->SF[0].W[6].bit[1];
	curGpsEphem.af1 = ((double) union_short.s) * pow(2.0,-43);
	 
	 //a_f0
	 union_int.c[0] = ubx.rxm_eph->SF[0].W[7].bit[0];
	 union_int.c[1] = ubx.rxm_eph->SF[0].W[7].bit[1];
	 union_int.c[2] = ubx.rxm_eph->SF[0].W[7].bit[2];
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
	 curGpsEphem.af0 = ((double) union_int.i) * pow(2.0,-31);
	 
	 //M_0
	 union_int.c[0] = ubx.rxm_eph->SF[1].W[2].bit[0];
	 union_int.c[1] = ubx.rxm_eph->SF[1].W[2].bit[1];
	 union_int.c[2] = ubx.rxm_eph->SF[1].W[2].bit[2];
	 union_int.c[3] = ubx.rxm_eph->SF[1].W[1].bit[0];
	 curGpsEphem.anrtime = ((double) union_int.i) * pow(2.0,-31) * PI;
	 
	 //deltan
	 union_short.c[0] = ubx.rxm_eph->SF[1].W[1].bit[1];
	 union_short.c[1] = ubx.rxm_eph->SF[1].W[1].bit[2];
	 curGpsEphem.dN = ((double) union_short.s) * pow(2.0,-43) * PI;
	 
	 //ecc
	 union_unsigned_int.c[0] = ubx.rxm_eph->SF[1].W[4].bit[0];
	 union_unsigned_int.c[1] = ubx.rxm_eph->SF[1].W[4].bit[1];
	 union_unsigned_int.c[2] = ubx.rxm_eph->SF[1].W[4].bit[2];
	 union_unsigned_int.c[3] = ubx.rxm_eph->SF[1].W[3].bit[0];
	 curGpsEphem.ecc = ((double) union_unsigned_int.i) * pow(2.0,-33);
	 
	 //sqrtA
	 union_unsigned_int.c[0] = ubx.rxm_eph->SF[1].W[6].bit[0];
	 union_unsigned_int.c[1] = ubx.rxm_eph->SF[1].W[6].bit[1];
	 union_unsigned_int.c[2] = ubx.rxm_eph->SF[1].W[6].bit[2];
	 union_unsigned_int.c[3] = ubx.rxm_eph->SF[1].W[5].bit[0];
	 curGpsEphem.majaxis = ((double) union_unsigned_int.i) * pow(2.0,-19);
	 
	 //OMEGA_0
	 union_int.c[0] = ubx.rxm_eph->SF[2].W[1].bit[0];
	 union_int.c[1] = ubx.rxm_eph->SF[2].W[1].bit[1];
	 union_int.c[2] = ubx.rxm_eph->SF[2].W[1].bit[2];
	 union_int.c[3] = ubx.rxm_eph->SF[2].W[0].bit[0];
	 curGpsEphem.wo = ((double) union_int.i) * pow(2.0,-31) * PI;
	 
	 //i_0
	 union_int.c[0] = ubx.rxm_eph->SF[2].W[3].bit[0];
	 union_int.c[1] = ubx.rxm_eph->SF[2].W[3].bit[1];
	 union_int.c[2] = ubx.rxm_eph->SF[2].W[3].bit[2];
	 union_int.c[3] = ubx.rxm_eph->SF[2].W[2].bit[0];
	 curGpsEphem.ia = ((double) union_int.i) * pow(2.0,-31) * PI;
	 
	 //omega
	 union_int.c[0] = ubx.rxm_eph->SF[2].W[5].bit[0];
	 union_int.c[1] = ubx.rxm_eph->SF[2].W[5].bit[1];
	 union_int.c[2] = ubx.rxm_eph->SF[2].W[5].bit[2];
	 union_int.c[3] = ubx.rxm_eph->SF[2].W[4].bit[0];
	 curGpsEphem.omega = ((double) union_int.i) * pow(2.0,-31) * PI;
	 
	 //OMEGADOT
	 union_int.c[0] = ubx.rxm_eph->SF[2].W[6].bit[0];
	 union_int.c[1] = ubx.rxm_eph->SF[2].W[6].bit[1];
	 union_int.c[2] = ubx.rxm_eph->SF[2].W[6].bit[2];
	 
	 if ((union_int.c[2]>>7)&0x01)
	 {
	 union_int.c[3] = 0xff;
	 }
	 else
	 {
	 union_int.c[3] = 0x00;
	 }
	 curGpsEphem.dwo = ((double) union_int.i) * pow(2.0,-43) * PI;
	 
	 //IDOT
	 union_short.c[0] = ubx.rxm_eph->SF[2].W[7].bit[0];
	 union_short.c[1] = ubx.rxm_eph->SF[2].W[7].bit[1];
	 union_short.s = union_short.s>>2;
	 if ((union_short.c[1]>>5)&0x01)
	 {
	 union_int.c[1] = union_short.c[1] | 0xC0;
	 }
	 else
	 {
	 union_int.c[1] = union_short.c[1] & 0x3F;
	 }
	 curGpsEphem.dia = ((double) union_short.s) * pow(2.0,-43) * PI;
	 
	 //C_uc
	 union_short.c[0] = ubx.rxm_eph->SF[1].W[3].bit[1];
	 union_short.c[1] = ubx.rxm_eph->SF[1].W[3].bit[2];
	 curGpsEphem.cuc = ((double) union_short.s) * pow(2.0,-29);
	 
	 //C_us
	 union_short.c[0] = ubx.rxm_eph->SF[1].W[5].bit[1];
	 union_short.c[1] = ubx.rxm_eph->SF[1].W[5].bit[2];
	 curGpsEphem.cus = ((double) union_short.s) * pow(2.0,-29);
	 
	 //C_rc
	 union_short.c[0] = ubx.rxm_eph->SF[2].W[4].bit[1];
	 union_short.c[1] = ubx.rxm_eph->SF[2].W[4].bit[2];
	 curGpsEphem.crc = ((double) union_short.s) * pow(2.0,-5);
	 
	 //C_rs
	 union_short.c[0] = ubx.rxm_eph->SF[1].W[0].bit[0];
	 union_short.c[1] = ubx.rxm_eph->SF[1].W[0].bit[1];
	 curGpsEphem.crs = ((double) union_short.s) * pow(2.0,-5);
	 
	 //C_ic
	 union_short.c[0] = ubx.rxm_eph->SF[2].W[0].bit[1];
	 union_short.c[1] = ubx.rxm_eph->SF[2].W[0].bit[2];
	 curGpsEphem.cic = ((double) union_short.s) * pow(2.0,-29);
	 
	 //C_is
	 union_short.c[0] = ubx.rxm_eph->SF[2].W[2].bit[1];
	 union_short.c[1] = ubx.rxm_eph->SF[2].W[2].bit[2];
	 curGpsEphem.cis = ((double) union_short.s) * pow(2.0,-29);
	 
	 //t_oe
	 union_unsigned_short.c[0] = ubx.rxm_eph->SF[1].W[7].bit[1];
	 union_unsigned_short.c[1] = ubx.rxm_eph->SF[1].W[7].bit[2];
	 curGpsEphem.toe = ((double) union_unsigned_short.s) * pow(2.0,4);
    */
}


