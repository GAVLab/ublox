#include "ublox/ublox.h"
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
using namespace novatel;

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

void DefaultNavPosLlhCallback(Position best_position, double time_stamp){
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

ublox::~ublox()
{
	
}


bool ublox::RequestLogOnChanged(string logID) {

	cout << "RequestLogOnChanged " << endl;

	if(logID == "RXMEPH"){
	
		cout << "constructing Ephem Request message" << endl;
	
		char check[2];
		unsigned int length = 6;
		unsigned int totLength = 8;
	
		char data[8];
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

		return (SendString(data,totLength)==totLength);				
	
	}
	else{

		return false;

	}
	return true;

}

bool ublox::RequestLogOnTime(string logID, string period) {
	
	MOOSTrace("inside requestlog on time\n");
	MOOSTrace(logID);
	
	if(logID == "RXMRAW"){
	
		char check[2];
		unsigned int length = 9;
		unsigned int totLength = 11;
		char data[11];

		// Header
		data[0] = 0xB5;
		data[1] = 0x62;
		// ID
		data[2] = 0x06;
		data[3] = 0x01;
		// Length
		data[4] = 3;
		data[5] = 0;
		// Message Class
		data[6] = 0x02;
		// Message ID
		data[7] = 0x10;
		// Rate
		data[8] = 0x01;

		calculateCheckSum(data,length,check);

		// Check Sum
		data[9] = check[0];
		data[10] = check[1];
		
		//for (unsigned int i=0; i<11; i++)
		//{
		//	cout << hex << (int)data[i] << dec << endl;
		//
		//}
		
		//return true;

		return (SendString(data,totLength)==totLength);				
	
	}
	else if(logID == "NAVSOL"){
	
		cout << "logID == NAVSOL" << endl;
	
		char check[2];
		unsigned int length = 9;
		unsigned int totLength = 11;
		char data[11];

		// Header
		data[0] = 0xB5;
		data[1] = 0x62;
		// ID
		data[2] = 0x06;
		data[3] = 0x01;
		// Length
		data[4] = 3;
		data[5] = 0;
		// Message Class
		data[6] = 0x01;
		// Message ID
		data[7] = 0x06;
		// Rate
		data[8] = 0x01;

		calculateCheckSum(data,length,check);

		// Check Sum
		data[9] = check[0];
		data[10] = check[1];
		
		//for (unsigned int i=0; i<11; i++)
		//{
		//	cout << hex << (int)data[i] << dec << endl;
		//
		//}

		return (SendString(data,totLength)==totLength);
		//return true;				
	
	}
	else if(logID == "NAVVELNED"){
	
		//cout << "logID == NAVSOL" << endl;
	
		char check[2];
		unsigned int length = 9;
		unsigned int totLength = 11;
		char data[11];

		// Header
		data[0] = 0xB5;
		data[1] = 0x62;
		// ID
		data[2] = 0x06;
		data[3] = 0x01;
		// Length
		data[4] = 3;
		data[5] = 0;
		// Message Class
		data[6] = 0x01;
		// Message ID
		data[7] = 0x12;
		// Rate
		data[8] = 0x01;

		calculateCheckSum(data,length,check);

		// Check Sum
		data[9] = check[0];
		data[10] = check[1];
		
		//for (unsigned int i=0; i<11; i++)
		//{
		//	cout << hex << (int)data[i] << dec << endl;
		//
		//}

		return (SendString(data,totLength)==totLength);
		//return true;				
	
	}
	else{

		return false;

	}
	return true;
	
}


void ublox::BufferIncomingData(unsigned char *msg, unsigned int length)
{
	//MOOSTrace("Inside BufferIncomingData\n");
	//cout << length << endl;
    //cout << 0 << ": " << dec << (int)msg[0] << endl;
	// add incoming data to buffer
	for (unsigned int i=0; i<length; i++)
	{
		//cout << i << ": " << hex << (int)msg[i] << dec << endl;
		// make sure bufIndex is not larger than buffer
		if (bufIndex>=MAX_NOUT_SIZE)
		{
			bufIndex=0;
			MOOSTrace("ublox: Overflowed receive buffer. Reset");
		}

		//cout << "bufIndex = " << bufIndex << endl;
		//if (bufIndex > 200)
		//{
		//	exit(-1);
		//}
		if (bufIndex==0)
		{	// looking for beginning of message
			if (msg[i]==0xB5)
			{	// beginning of msg found - add to buffer
				//cout << "got first bit" << endl;				
				dataBuf[bufIndex++]=msg[i];
				bytesRemaining=0;
			}	// end if (msg[i]
		} // end if (bufIndex==0)
		else if (bufIndex==1)
		{	// verify 2nd character of header
			if (msg[i]==0x62)
			{	// 2nd byte ok - add to buffer
				//cout << " got second synch bit" << endl;
				dataBuf[bufIndex++]=msg[i];
			}
			else
			{
				// start looking for new message again
				bufIndex=0;
				bytesRemaining=0;
				readingACK=false;
			} // end if (msg[i]==0x62)
		}	// end else if (bufIndex==1)
		else if (bufIndex==2)
		{	//look for ack
			if (msg[i]==0x05)
			{
				//cout << "got ACK " << endl;
				bufIndex = 0;
				bytesRemaining=0;
				readingACK = false;
			}
			else
			{
				dataBuf[bufIndex++]=msg[i];
				readingACK = false;
			}
		}
		else if (bufIndex==3)
		{	
			
			// msg[i] and msg[i-1] define message ID
			dataBuf[bufIndex++]=msg[i];
			// length of header is in byte 4
			
			//printHex(reinterpret_cast < char * > (dataBuf),4);
			
			
			msgID=((dataBuf[bufIndex-2])<<8)+dataBuf[bufIndex-1];
			//cout << "msgID = " << msgID << endl;
		}
		else if (bufIndex==5)
		{	
			// add byte to buffer
			dataBuf[bufIndex++]=msg[i];
			// length of message (payload + 2 byte check sum)
			bytesRemaining = ((dataBuf[bufIndex-1])<<8)+dataBuf[bufIndex-2]+2;
			
			//cout << "bytesRemaining = " << bytesRemaining << endl;
			
			///cout << msgID << endl;
		}
		else if (bufIndex==6)
		{	// set number of bytes
			dataBuf[bufIndex++]=msg[i];
			bytesRemaining--;
		}
		else if (bytesRemaining==1)
		{	// add last byte and parse
			dataBuf[bufIndex++]=msg[i];
			//cout << " msgID = " << msgID << endl;
			ParseLog(dataBuf+4,msgID);
			// reset counters
			bufIndex=0;
			bytesRemaining=0;
			//cout << "Message Done." << endl;
		}  // end else if (bytesRemaining==1)
		else
		{	// add data to buffer
			dataBuf[bufIndex++]=msg[i];
			bytesRemaining--;
		}
	}	// end for
}

bool ublox::WaitForAck(int timeout) {
	// waits timeout # of seconds for an acknowledgement from the receiver
	// returns true if one is received, false for a timeout
	if (ackReceived)
		return true;
	else {
		for (int ii=0; ii<timeout; ii++){
			// sleep for 1 second
			MOOSPause(1000);
			// check again
			if (ackReceived)
				return true;
		}
		return false;
	}
}

void ublox::ParseLog(unsigned char *log, unsigned int logID)
{

	int length;
	//cout << "ParseLog Started" << endl;
	//cout << "logID = " << logID << endl;
	//double publishTime;
	switch (logID)
	{

		case navSolID:

			memcpy(&curNavSolData, log+2, sizeof(curNavSolData));
			//PublishNavSolToDB(curNavSolData);
			break;
		case ephID:

			
			length = ((log[1])<<8) + log[0];
			if (length == 0x68)
			{
							
				ubx.rxm_eph = (struct s_rxm_eph*) log;
			//	cout << "ubx length " << ubx.rxm_eph->len << endl;
			//	cout << "ubx prn " << ubx.rxm_eph->svprn << endl;
				Parse_rxm_eph();
				curGpsEphem.prn=ubx.rxm_eph->svprn;
			//	PublishGpsEphemToDB(curGpsEphem,MOOSTime());
				
			}
			
			break;
		case rangeID:
			
			//cout << "sizeoff range log Struct + " << sizeof(curRawData) << endl;
			
			length = ((log[1])<<8) + log[0];
			//int numSv = log[8];
			//cout << "numSv from log =  " << numSv << endl;
			//cout << "length of Range message = " << length << endl;
			memcpy(&curRawData, log+2, length);
			//PublishRangeToDB(curRawData);
			break;	
		case navVelNedID:

			//cout << "sizeoff NavSol Struct + " << sizeof(curNavSolData) << endl;
			//int length;
			//length = ((log[1])<<8)+log[0];
			//cout << "length of Nav message = " << length << endl;
			memcpy(&curNavVelNed, log+2, sizeof(curNavVelNed));
			//PublishNavVelNedToDB(curNavVelNed);
			//cout << "Something Good NAVSOL" << endl;
			break;

	}
}
b

void ublox::calculateCheckSum(char* in, unsigned int length, char* out)
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

void ublox::Parse_rxm_eph()
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
	
}


