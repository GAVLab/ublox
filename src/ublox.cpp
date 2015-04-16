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

inline void printHex(char *data, int length) {
    for (int i = 0; i < length; ++i) {
        printf("0x%.2X ", (unsigned) (unsigned char) data[i]);
    }
    printf("\n");
}

/*!
 * Default callback method for time stamping data.  Used if a
 * user callback is not set.  Returns the current time from the
 * CPU clock as the number of seconds from Jan 1, 1970
 */
double DefaultGetTime() {
    boost::posix_time::ptime present_time(
            boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return duration.total_seconds();
}

void DefaultAcknowledgementHandler() {
    //std::cout << "Acknowledgment received." << std::endl;
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

inline void DefaultPortSettingsCallback(CfgPrt port_settings,
        double time_stamp) {
    std::cout << "CFG-PRT:" << std::endl;
}

inline void DefaultConfigureNavigationParametersCallback(CfgNav5 cfg_nav,
        double time_stamp) {
    std::cout << "CFG-NAV5:" << std::endl;
}

inline void DefaultErrorMsgCallback(const std::string &msg) {
    std::cout << "Ublox Error: " << msg << std::endl;
}

inline void DefaultNavSolCallback(ublox::NavSol nav_sol, double time_stamp) {
    std::cout << "NAV-SOL: " << endl;
}

inline void DefaultNavStatusCallback(ublox::NavStatus nav_status, double time_stamp) {
    std::cout << "GPS Fix Type: ";
    if (nav_status.fixtype == 0x00) {
        std::cout << "No Fix" << std::endl;
        std::cout << "TTFF: " << " none ms" << std::endl;
        std::cout << "Milliseconds since Startup/Reset: " << nav_status.msss
                << std::endl;
    } else if (nav_status.fixtype == 0x01) {
        std::cout << "Dead Reckoning Only" << std::endl;
    } else if (nav_status.fixtype == 0x02) {
        std::cout << "2D Fix" << std::endl;
    } else if (nav_status.fixtype == 0x03) {
        std::cout << "3D Fix" << std::endl;
    } else if (nav_status.fixtype == 0x04) {
        std::cout << "GPS + Dead Reckoning" << std::endl;
    } else if (nav_status.fixtype == 0x05) {
        std::cout << "Time Only" << std::endl;
    } else {
        std::cout << std::endl;
    }

    if (nav_status.fixtype != 0x00) {
        std::cout << "TTFF: " << (nav_status.ttff / 1000.) << " sec"
                << std::endl;
        std::cout << "Milliseconds since Startup/Reset: "
                << (nav_status.msss / 1000.) << " sec" << std::endl;
    }
}

inline void DefaultNavVelNedCallback(ublox::NavVelNed nav_vel_ned, double time_stamp) {
    std::cout << "NAV-VELNED: " << endl;
}

inline void DefaultNavSVInfoCallback(ublox::NavSVInfo nav_sv_info, double time_stamp) {
    std::cout << "NAV-SVINFO: " << endl;
}

inline void DefaultNavGPSTimeCallback(ublox::NavGPSTime nav_gps_time,
        double time_stamp) {
    std::cout << "NAV-GPSTIME: " << endl;
}

inline void DefaultNavUTCTimeCallback(ublox::NavUTCTime nav_utc_time,
        double time_stamp) {
    std::cout << "NAV-UTCTIME: " << endl;
}

inline void DefaultNavDOPCallback(ublox::NavDOP nav_dop, double time_stamp) {
    std::cout << "NAV-DOP: " << endl;
}

inline void DefaultNavDGPSCallback(ublox::NavDGPS nav_dgps, double time_stamp) {
    std::cout << "NAV-DGPS: " << endl;
}

inline void DefaultNavClockCallback(ublox::NavClock nav_clock, double time_stamp) {
    std::cout << "NAV-CLK: " << endl;
}

inline void DefaultNavPosLlhCallback(ublox::NavPosLLH nav_position, double time_stamp){
    /*std:: cout << "NAV-POSLLH: " << endl <<
                  "  GPS milliseconds: " << nav_position.iTOW << std::endl <<
                  "  Latitude: " << nav_position.latitude_scaled << std::endl <<
                  "  Longitude: " << nav_position.longitude_scaled << std::endl <<
                  "  Height: " << nav_position.height << std::endl << std::endl;*/
}

inline void DefaultAidEphCallback(ublox::EphemSV eph_sv, double time_stamp) {
    std::cout << "AID-EPH: " << std::endl;
}

inline void DefaultAidAlmCallback(ublox::AlmSV alm_sv, double time_stamp) {
    std::cout << "AID-ALM: " << std::endl;

}

inline void DefaultAidHuiCallback(ublox::AidHui aid_hui, double time_stamp) {
    std::cout << "AID-HUI: " << std::endl;
}

inline void DefaultAidIniCallback(ublox::AidIni aid_ini, double time_stamp) {
    std::cout << "AID-INI: " << std::endl;
}

inline void DefaultRxmRawCallback(ublox::RawMeas raw_meas, double time_stamp) {
    std::cout << "RXM-RAW: " << std::endl;
}

inline void DefaultRxmSubframeCallback(ublox::SubframeData subframe, double time_stamp) {
    std::cout << "RXM-SFRB: " << std::endl;
}

inline void DefaultRxmSvsiCallback(ublox::SVStatus sv_stat, double time_stamp) {
    std::cout << "RXM-SVSI: " << std::endl;
}

inline void DefaultParsedEphemCallback(ublox::ParsedEphemData parsed_ephem_data,
        double time_stamp) {
    /*
    std::cout << "Parsed ephemeris: " << std::endl;
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
}

Ublox::Ublox() {
    serial_port_ = NULL;
    reading_status_ = false;
    time_handler_ = DefaultGetTime;
    handle_acknowledgement_ = DefaultAcknowledgementHandler;
    port_settings_callback_ = NULL;
    configure_navigation_parameters_callback_ = NULL;
    nav_pos_llh_callback_ = NULL;
    aid_eph_callback_ = NULL;
    aid_alm_callback_ = NULL;
    aid_hui_callback_ = NULL;
    aid_ini_callback_ = NULL;
    rxm_raw_callback_ = NULL;
    rxm_subframe_callback_ = NULL;
    rxm_svsi_callback_ = NULL;
    nav_sol_callback_ = NULL;
    nav_status_callback_ = DefaultNavStatusCallback;
    nav_vel_ned_callback_ = NULL;
    nav_sv_info_callback_ = NULL;
    nav_gps_time_callback_ = NULL;
    nav_utc_time_callback_ = NULL;
    nav_dop_callback_ = NULL;
    nav_dgps_callback_ = NULL;
    nav_clock_callback_ = NULL;
    log_debug_ = DefaultDebugMsgCallback;
    log_info_ = DefaultInfoMsgCallback;
    log_warning_ = DefaultWarningMsgCallback;
    log_error_ = DefaultErrorMsgCallback;
    parsed_ephem_callback_ = NULL;
    reading_acknowledgement_ = false;
    bytes_remaining_ = false;
    header_length_ = 0;
    msgID = 0;
    data_read_ = NULL;
    buffer_index_ = 0;
    read_timestamp_ = 0;
    parse_timestamp_ = 0;
    is_connected_ = false;
}

Ublox::~Ublox() {
    Disconnect();
}

bool Ublox::Connect(std::string port, int baudrate) {
    //serial_port_ = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(1000));
    serial::Timeout my_timeout(100, 1000, 0, 1000, 0);
    try {
        serial_port_ = new serial::Serial(port, baudrate, my_timeout);


		if (!serial_port_->isOpen()) {
			std::stringstream output;
			output << "Serial port: " << port << " failed to open.";
			log_error_(output.str());
			delete serial_port_;
			serial_port_ = NULL;
			is_connected_ = false;
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
		if (!Ping()) {
			std::stringstream output;
			output << "Ublox GPS not found on port: " << port << std::endl;
			log_error_(output.str());
			delete serial_port_;
			serial_port_ = NULL;
			is_connected_ = false;
			return false;
		}


    } catch (std::exception e) {
           std::stringstream output;
           output << "Failed to open port " << port << "  Err: " << e.what();
           log_error_(output.str());
           serial_port_ = NULL;
           is_connected_ = false;
           return false;
       }

    // start reading
    StartReading();
    is_connected_ = true;
    return true;

}

bool Ublox::Ping(int num_attempts) {
    try {
        while ((num_attempts--) > 0) {
            log_info_("Searching for Ublox receiver...");
            // request version information

            // ask for version
            PollMessage(MSG_CLASS_MON, MSG_ID_MON_VER);

            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

            unsigned char result[5000];
            size_t bytes_read;
            bytes_read = serial_port_->read(result, 5000);

            //std::cout << "bytes read: " << (int)bytes_read << std::endl;
            //std::cout << dec << result << std::endl;

            if (bytes_read < 8) {
                stringstream output;
                output << "Only read " << bytes_read
                        << " bytes in response to ping.";
                log_debug_(output.str());
                continue;
            }

            uint16_t length;
            // search through result for version message
            for (int ii = 0; ii < (bytes_read - 8); ii++) {
                //std::cout << hex << (unsigned int)result[ii] << std::endl;
                if (result[ii] == UBX_SYNC_BYTE_1) {
                    if (result[ii + 1] != UBX_SYNC_BYTE_2)
                        continue;
                    if (result[ii + 2] != MSG_CLASS_MON)
                        continue;
                    if (result[ii + 3] != MSG_ID_MON_VER)
                        continue;
                    //std::cout << "length1:" << hex << (unsigned int)result[ii+4] << std::endl;
                    //std::cout << "length2:" << hex << (unsigned int)result[ii+5] << std::endl;
                    length = (result[ii + 4]) + (result[ii + 5] << 8);
                    if (length < 40) {
                        log_debug_("Incomplete version message received");
                        //    //return false;
                        continue;
                    }

                    string sw_version;
                    string hw_version;
                    string rom_version;
                    sw_version.append((char*) (result + 6));
                    hw_version.append((char*) (result + 36));
                    //rom_version.append((char*)(result+46));
                    log_info_("Ublox receiver found.");
                    log_info_("Software Version: " + sw_version);
                    log_info_("Hardware Version: " + hw_version);
                    //log_info_("ROM Version: " + rom_version);
                    return true;
                }
            }
            stringstream output;
            output << "Read " << bytes_read
                    << " bytes, but version message not found.";
            log_debug_(output.str());

        }
    } catch (exception &e) {
        std::stringstream output;
        output << "Error pinging receiver: " << e.what();
        log_error_(output.str());
        return false;
    }

    return false;
}

void Ublox::Disconnect() {
	try {
		if (reading_status_) {
			StopReading();
			// TODO: wait here for reading to stop
		}
		if (serial_port_ != NULL) {
			if (serial_port_->isOpen())
				serial_port_->close();
			delete serial_port_;
			serial_port_ = NULL;
		}
	} catch (std::exception &e) {
		std::stringstream output;
				output << "Error disconnecting from ublox: " << e.what();
				log_error_(output.str());
	}
}

void Ublox::StartReading() {
	try {
		// create thread to read from sensor
		reading_status_ = true;
		read_thread_ptr_ = boost::shared_ptr<boost::thread>(
				new boost::thread(boost::bind(&Ublox::ReadSerialPort, this)));
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error starting ublox read thread: " << e.what();
		log_error_(output.str());
	}
}

void Ublox::StopReading() {
    reading_status_ = false;
}

void Ublox::ReadSerialPort() {
    uint8_t buffer[MAX_NOUT_SIZE];
    size_t len;

    // continuously read data from serial port
    while (reading_status_) {
        // read data
        try {
            len = serial_port_->read(buffer, MAX_NOUT_SIZE);
        } catch (exception &e) {
            stringstream output;
            output << "Error reading serial port: " << e.what();
            log_info_(output.str());
            Disconnect();
            return;
        }
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
bool Ublox::PollMessage(uint8_t class_id, uint8_t msg_id) {
	try {
		uint8_t message[8];

		message[0]=UBX_SYNC_BYTE_1;        // sync 1
		message[1]=UBX_SYNC_BYTE_2;        // sync 2
		message[2]=class_id;
		message[3]=msg_id;
		message[4]=0;           // length 1
		message[5]=0;           // length 2
		message[6]=0;           // checksum 1
		message[7]=0;           // checksum 2

		uint8_t* msg_ptr = (uint8_t*) &message;

		calculateCheckSum(msg_ptr + 2, 4, msg_ptr + 6);

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		  size_t bytes_written = serial_port_->write(message, 8);
          return bytes_written == 8;
        } else {
            log_error_("Unable to send poll message. Serial port not open.");
            return false;
        }

	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending ublox poll message: " << e.what();
		log_error_(output.str());
		return 0;
	}
}

// Poll Message used to request for one SV
bool Ublox::PollMessageIndSV(uint8_t class_id, uint8_t msg_id, uint8_t svid) {
    try {
		uint8_t message[9];

		message[0] = UBX_SYNC_BYTE_1;        // sync 1
		message[1] = UBX_SYNC_BYTE_2;        // sync 2
		message[2] = class_id;
		message[3] = msg_id;
		message[4] = 1;           // length 1
		message[5] = 0;           // length 2
		message[6] = svid;        // Payload
		message[7] = 0;           // checksum 1
		message[8] = 0;           // checksum 2

		uint8_t* msg_ptr = (uint8_t*) &message;
		calculateCheckSum(msg_ptr + 2, 5, msg_ptr + 7);

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		    size_t bytes_written = serial_port_->write(msg_ptr, 9);
            return bytes_written == 9;
        } else {
            log_error_("Unable to send poll ind. sv. message. Serial port not open.");
            return false;
        }

    } catch (std::exception &e) {
		std::stringstream output;
		output << "Error polling individual svs: " << e.what();
		log_error_(output.str());
		return 0;
	}
}

// (CFG-NAV5) Polls current navigation algorithms parameters
bool Ublox::PollNavigationParamterConfiguration() {
    log_info_("Polling for CFG-NAV5..");
    return PollMessage(MSG_CLASS_CFG, MSG_ID_CFG_NAV5);
}

// (AID-EPH) Polls for Ephemeris data
bool Ublox::PollEphem(int8_t svid) {

    if (svid < -1) {
        log_error_("Error in PollEphem: Invalid input 'svid'");
        return 0;
    } else if (svid == -1) { // Requests Ephemerides for all SVs
        log_debug_("Polling for all Ephemerides..");
        return PollMessage(MSG_CLASS_AID, MSG_ID_AID_EPH);
    } else if (svid > 0) { // Requests Ephemeris for a single SV
        stringstream output;
        output << "Polling for SV# " << (int) svid << " Ephemeris..";
        log_debug_(output.str());
        return PollMessageIndSV(MSG_CLASS_AID, MSG_ID_AID_EPH, (uint8_t) svid);
    } else {
        log_error_("Error in PollEphem: Invalid input 'svid'");
        return 0;
    }
}

// (AID-ALM) Polls for Almanac Data
bool Ublox::PollAlmanac(int8_t svid) {

    if (svid < -1) {
        log_error_("Error in PollAlmanac: Invalid input 'svid'");
        return 0;
    } else if (svid == -1) { // Requests Almanac Data for all SVs
        log_debug_("Polling for all Almanac Data..");
        return PollMessage(MSG_CLASS_AID, MSG_ID_AID_ALM);
    } else if (svid > 0) { // Requests Almanac Data for a single SV
        stringstream output;
        output << "Polling for SV# " << (int) svid << " Almanac Data..";
        log_debug_(output.str());
        return PollMessageIndSV(MSG_CLASS_AID, MSG_ID_AID_ALM, (uint8_t) svid);
    } else {
        log_error_("Error in PollAlmanac: Invalid input 'svid'");
        return 0;
    }
}

// (AID-HUI) Polls GPS Health, UTC and Ionospheric Parameters
bool Ublox::PollHUI() {
    log_debug_("Polling for AID-HUI..");
    return PollMessage(MSG_CLASS_AID, MSG_ID_AID_HUI);
}

// (AID-INI) Polls for Receiver Position, Time, Frequency, and Clock Drift
bool Ublox::PollIniAid() {
    log_debug_("Polling for AID-INI..");
    return PollMessage(MSG_CLASS_AID, MSG_ID_AID_INI);
}

// (AID-DATA) Polls for All AID Data (-INI, -HUI, -EPH, -ALM)
bool Ublox::PollAllAidData() {
    log_debug_("Polling for AID-HUI, AID-INI, AID-EPH, & AID-ALM..");
    return PollMessage(MSG_CLASS_AID, MSG_ID_AID_DATA);
}

// (RXM-RAW) Polls for Raw DGPS data
bool Ublox::PollRawDgpsData() {
    log_debug_("Polling for RXM-RAW..");
    return PollMessage(MSG_CLASS_RXM, MSG_ID_RXM_RAW);
}

// (RXM-SVSI) Polls for Satellite Status Info
bool Ublox::PollSVStatus() {
    log_debug_("Polling for RXM-SVSI..");
    return PollMessage(MSG_CLASS_RXM, MSG_ID_RXM_SVSI);
}

// (NAV-SVINFO) Polls for Space Vehicle Information
bool Ublox::PollSVInfo() {
    log_debug_("Polling for NAV-SVINFO..");
    return PollMessage(MSG_CLASS_NAV, MSG_ID_NAV_SVINFO);
}

// (NAV-STATUS) Polls for Receiver Navigation Status
bool Ublox::PollNavStatus() {
    log_debug_("Polling for Receiver NAV-STATUS..");
    return PollMessage(MSG_CLASS_NAV, MSG_ID_NAV_STATUS);
}

////////////////////////////////////////////////////////
// (CFG) Configuration Messages
////////////////////////////////////////////////////////
// Receiver Reset
bool Ublox::Reset(uint16_t nav_bbr_mask, uint8_t reset_mode) {
	try {
        ublox::CfgRst message;

		message.header.sync1 = UBX_SYNC_BYTE_1;
		message.header.sync2 = UBX_SYNC_BYTE_2;
		message.header.message_class = MSG_CLASS_CFG;
		message.header.message_id = MSG_ID_CFG_RST;
		message.header.payload_length = 4;

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

		unsigned char* msg_ptr = (unsigned char*) &message;
		calculateCheckSum(msg_ptr + 2, 8, message.checksum);
        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		  return serial_port_->write(msg_ptr, sizeof(message)) == sizeof(message);
        } else {
            log_error_("Unable to send reset command. Serial port not open.");
            return false;
        }
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error resetting ublox: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Receiver Reset Messages - Force Cold Start
bool Ublox::ResetToColdStart(uint8_t reset_mode) {
    log_info_("Receiver reset to cold start state.");
    return Reset(0xFFFF, reset_mode);
}

// Receiver Reset Messages - Force Warm Start
bool Ublox::ResetToWarmStart() {
    log_info_("Receiver reset to warm start state.");
    return Reset(0x0001, 0x02);
}

// Receiver Reset Messages - Force Hot Start
bool Ublox::ResetToHotStart() {
    log_info_("Receiver reset to hot start state.");
    return Reset(0x0000, 0x02);
}

// (CFG-NAV5) Cofigure Navigation Algorithm Parameters
bool Ublox::ConfigureNavigationParameters(uint8_t dynamic_model, uint8_t fix_mode){
	try {
        ublox::CfgNav5 message;

		message.header.sync1 = UBX_SYNC_BYTE_1;
		message.header.sync2 = UBX_SYNC_BYTE_2;
		message.header.message_class = MSG_CLASS_CFG;
		message.header.message_id = MSG_ID_CFG_NAV5;
		message.header.payload_length = 36;

		message.mask = 0b00000101;
		message.dynamic_model = dynamic_model;
		message.fix_mode = fix_mode;
		message.fixed_altitude = 0;
		message.fixed_altitude_variance = 0;
		message.min_elevation = 0;
		message.dead_reckoning_limit = 0;
		message.pdop = 0;
		message.tdop = 0;
		message.pos_accuracy_mask = 0;
		message.time_accuracy_mask = 0;
		message.static_hold_threshold = 0;
		message.dgps_timeout = 0;
		message.reserved2 = 0;
		message.reserved3 = 0;
		message.reserved4 = 0;

		unsigned char* msg_ptr = (unsigned char*) &message;
		calculateCheckSum(msg_ptr + 2, 36+4, message.checksum);

		return serial_port_->write(msg_ptr, sizeof(message)) == sizeof(message);
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error configuring ublox navigation parameters: " << e.what();
		log_error_(output.str());
		return false;
	}
}

bool Ublox::SbasOff() { //< Tell ublox not to use SBAS SVs
    try {
        ublox::CfgSbas message;
        message.header.sync1 = UBX_SYNC_BYTE_1;
        message.header.sync2 = UBX_SYNC_BYTE_2;
        message.header.message_class = MSG_CLASS_CFG;
        message.header.message_id = MSG_ID_CFG_SBAS;
        message.header.payload_length = 8;

        message.mode = 0b0;
        message.usage = 0b0;
        message.maxSBAS = 0b0;
        message.scanmode2 = 0b0;
        message.scanmode1 = 0b0;

        unsigned char* msg_ptr = (unsigned char*) &message;
        calculateCheckSum(msg_ptr + 2, 7, message.checksum);

        return serial_port_->write(msg_ptr, sizeof(message)) == sizeof(message);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Ublox::SbasOff() " << e.what();
        log_error_(output.str());
        return false;
    }
}

bool Ublox::SbasOn() { //< Tell ublox not to use SBAS SVs
    try {
        ublox::CfgSbas message;
        message.header.sync1 = UBX_SYNC_BYTE_1;
        message.header.sync2 = UBX_SYNC_BYTE_2;
        message.header.message_class = MSG_CLASS_CFG;
        message.header.message_id = MSG_ID_CFG_SBAS;
        message.header.payload_length = 8;

        message.mode = 0b01;
        message.usage = 0b0111;
        message.maxSBAS = 3;
        message.scanmode2 = 0b0;
        message.scanmode1 = 0b0;

        unsigned char* msg_ptr = (unsigned char*) &message;
        calculateCheckSum(msg_ptr + 2, 7, message.checksum);

        return serial_port_->write(msg_ptr, sizeof(message)) == sizeof(message);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Ublox::SbasOff() " << e.what();
        log_error_(output.str());
        return false;
    }
}


// (CFG-MSG) Set message output rate for specified message
bool Ublox::ConfigureMessageRate(uint8_t class_id, uint8_t msg_id,
        uint8_t rate) {
	try {
        ublox::CfgMsgRate message;
		message.header.sync1 = UBX_SYNC_BYTE_1;
		message.header.sync2 = UBX_SYNC_BYTE_2;
		message.header.message_class = MSG_CLASS_CFG;
		message.header.message_id = MSG_ID_CFG_MSG;
		message.header.payload_length = 3;

		message.message_class = class_id;
		message.message_id = msg_id;
		message.rate = rate;

		unsigned char* msg_ptr = (unsigned char*) &message;
		calculateCheckSum(msg_ptr + 2, 7, message.checksum);

		return serial_port_->write(msg_ptr, sizeof(message)) == sizeof(message);
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error configuring ublox message rate: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Set Port Configuration
void Ublox::SetPortConfiguration(bool ubx_input, bool ubx_output,
        bool nmea_input, bool nmea_output) {
	try {
        ublox::CfgPrt message;
		//std::cout << sizeof(message) << std::endl;
		message.header.sync1 = UBX_SYNC_BYTE_1;
		message.header.sync2 = UBX_SYNC_BYTE_2;
		message.header.message_class = MSG_CLASS_CFG;
		message.header.message_id = MSG_ID_CFG_PRT;
		message.header.payload_length = 20;

		message.port_id = 3;          //Port identifier for USB Port (3)
		message.reserved = 0;
		message.tx_ready = 0;
		message.reserved2 = 0;
		message.reserved3 = 0;
		message.input_mask = 0;       // Specifies input protocols
		message.output_mask = 0;      // Specifies output protocols
		message.reserved4 = 0;
		message.reserved5 = 0;

		if (ubx_input)
			message.input_mask = message.input_mask | 0x0001;   // set first bit
		else
			message.input_mask = message.input_mask & 0xFFFE;   // clear first bit

		if (nmea_input)
			message.input_mask = message.input_mask | 0x0002;   // set second bit
		else
			message.input_mask = message.input_mask & 0xFFFD;   // clear second bit

		if (ubx_output)
			message.output_mask = message.output_mask | 0x0001;   // set first bit
		else
			message.output_mask = message.output_mask & 0xFFFE;   // clear first bit

		if (nmea_output)
			message.output_mask = message.output_mask | 0x0002;   // set second bit
		else
			message.output_mask = message.output_mask & 0xFFFD;  // clear second bit

		unsigned char* msg_ptr = (unsigned char*) &message;
		calculateCheckSum(msg_ptr + 2, 27, message.checksum);

		log_info_("Set Port Settings Message Sent");

		//printHex((char*) &message, sizeof(message));

		serial_port_->write(msg_ptr, sizeof(message));
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error configuring ublox port: " << e.what();
		log_error_(output.str());
	}
}

// Poll Port Configuration
void Ublox::PollPortConfiguration(uint8_t port_identifier)
{ // Port identifier = 3 for USB (default value if left blank)
  //                 = 1 or 2 for UART
	try {
		uint8_t message[9];
		message[0]=UBX_SYNC_BYTE_1;
		message[1]=UBX_SYNC_BYTE_2;
		message[2]=MSG_CLASS_CFG;
		message[3]=MSG_ID_CFG_PRT;
		message[4]=1;
		message[5]=0;
		message[6]=port_identifier;         //Port identifier for USB Port (3)
		message[7]=0;                       // Checksum A
		message[8]=0;                       // Checksum B

		unsigned char* msg_ptr = (unsigned char*)&message;
		calculateCheckSum(msg_ptr+2,5,msg_ptr+7);

		serial_port_->write(msg_ptr, sizeof(message));
		log_info_("Polling for Port Protocol Configuration.");
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error polling ublox port configuration: " << e.what();
		log_error_(output.str());
	}
}

//////////////////////////////////////////////////////////////
// Functions to  Aiding Data to Receiver
//////////////////////////////////////////////////////////////
// Send Message
bool Ublox::SendMessage(uint8_t* msg_ptr, size_t length)
{
	try {
		stringstream output1;
		//std::cout << length << std::endl;
		//std::cout << "Message Pointer" << endl;
		//printHex((char*) msg_ptr, length);
        size_t bytes_written;

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		  bytes_written=serial_port_->write(msg_ptr, length);
        } else {
            log_error_("Unable to send message. Serial port not open.");
            return false;
        }
		// check that full message was sent to serial port
		if (bytes_written == length) {
			return true;
		}
		else {
			log_error_("Full message was not sent over serial port.");
			output1 << "Attempted to send " << length << "bytes. " << bytes_written << " bytes sent.";
			log_error_(output1.str());
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending ublox message: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Send AID-INI to Receiver
bool Ublox::SendAidIni(AidIni ini)
{  
    //stringstream output;
	try {
		unsigned char* msg_ptr = (unsigned char*)&ini;
		calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_INI + 4,
							ini.checksum);

		// Check that provided ini message is correct size before sending
		if (sizeof(ini) == FULL_LENGTH_AID_INI)
		{
			bool sent_ini = SendMessage(msg_ptr, FULL_LENGTH_AID_INI);
			//output << "Sending AID-INI to receiver.";
			//log_debug_(output.str());
			return true;
		}
		else
		{
			//output << "Provided AID-INI message not of correct length.";
			//log_error_(output.str());
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending aid ini data to ublox: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Send AID-EPH to Receiver
bool Ublox::SendAidEphem(Ephemerides ephems)
{
	try {
		bool sent_ephem [32];

		for (uint8_t prn_index = 1; prn_index <= 32; prn_index++) {
			//stringstream output;
			if (ephems.ephemsv[prn_index].header.payload_length == PAYLOAD_LENGTH_AID_EPH_WITH_DATA) {
				//output << "Sending AID-EPH for PRN # "
						//<< (int) ephems.ephemsv[prn_index].svprn << " ..";
				uint8_t* msg_ptr = (uint8_t*) &ephems.ephemsv[prn_index];
				calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_EPH_WITH_DATA + 4,
									ephems.ephemsv[prn_index].checksum);
				sent_ephem[prn_index] = SendMessage(msg_ptr, FULL_LENGTH_AID_EPH_WITH_DATA);

			} else { // not a full ephemeris message
				//output << "No AID-EPH data for PRN # " << (int) prn_index << " ..";
			}
			//log_debug_(output.str());
		}
		return true;
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending ephemeris data to ublox: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Send AID-ALM to Receiver
bool Ublox::SendAidAlm(Almanac almanac) {

	try {
		bool sent_alm [32];

		for (uint8_t prn_index = 1; prn_index <= 32; prn_index++) {
			//stringstream output;

			if (almanac.almsv[prn_index].header.payload_length == PAYLOAD_LENGTH_AID_ALM_WITH_DATA) {
				//output << "Sending AID-ALM for PRN # "
						//<< (int) almanac.almsv[prn_index].svprn << " ..";
				uint8_t* msg_ptr = (uint8_t*) &almanac.almsv[prn_index];
				calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_ALM_WITH_DATA + 4,
									almanac.almsv[prn_index].checksum);
				sent_alm[prn_index] = SendMessage(msg_ptr, FULL_LENGTH_AID_ALM_WITH_DATA);
			}
			else {
				//output << "No AID-ALM data for PRN # " << (int) prn_index << " ..";
			}
			//log_debug_(output.str());
		}
		return true;

	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending almanac data to ublox: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Send AID-HUI to Receiver
bool Ublox::SendAidHui(AidHui hui)
{
	try {
		//stringstream output;

		unsigned char* msg_ptr = (unsigned char*)&hui;
		calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_HUI + 4,
									hui.checksum);

		if (sizeof(hui) == FULL_LENGTH_AID_HUI)
		{

			bool sent_hui = SendMessage(msg_ptr, FULL_LENGTH_AID_HUI);
			//output << "Sending AID-HUI to receiver..";
			//log_info_(output.str());
			return true;
		}
		else
		{
			//output << "Provided AID-HUI message not of correct length.";
			//log_error_(output.str());
			return false;
		}
	}catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending hui data to ublox: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Send RXM-RAW to Receiver
// NOTE: needs fixing still
bool Ublox::SendRawMeas(RawMeas raw_meas)
{
	try {
		stringstream output;

		output << "Sending RXM-RAW to receiver..";
		log_info_(output.str());
		unsigned char* msg_ptr = (unsigned char*)&raw_meas;
		return SendMessage(msg_ptr, sizeof(raw_meas));
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending raw data to ublox: " << e.what();
		log_error_(output.str());
		return false;
	}
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
void Ublox::BufferIncomingData(uint8_t *msg, size_t length) {
    //MOOSTrace("Inside BufferIncomingData\n");
    //cout << length << endl;
    //cout << 0 << ": " << dec << (int)msg[0] << endl;
    // add incoming data to buffer

    //printHex(reinterpret_cast<char*>(msg),length);
	try {

		for (unsigned int i = 0; i < length; i++) {
			//cout << i << ": " << hex << (int)msg[i] << dec << endl;
			// make sure buffer_index_ is not larger than buffer
			if (buffer_index_ >= MAX_NOUT_SIZE) {
				buffer_index_ = 0;
				log_warning_(
						"Overflowed receiver buffer. See ublox.cpp BufferIncomingData");

			}
			//cout << "buffer_index_ = " << buffer_index_ << endl;

			if (buffer_index_ == 0) {	// looking for beginning of message
				if (msg[i] == UBX_SYNC_BYTE_1) {	// beginning of msg found - add to buffer
										//cout << "got first bit" << endl;
					data_buffer_[buffer_index_++] = msg[i];
					bytes_remaining_ = 0;
				}	// end if (msg[i]
			} // end if (buffer_index_==0)
			else if (buffer_index_ == 1) {	// verify 2nd character of header
				if (msg[i] == UBX_SYNC_BYTE_2) {	// 2nd byte ok - add to buffer
										//cout << " got second synch bit" << endl;
					data_buffer_[buffer_index_++] = msg[i];
				} else {
					// start looking for new message again
					buffer_index_ = 0;
					bytes_remaining_ = 0;
					//readingACK=false;
				} // end if (msg[i]==UBX_SYNC_BYTE_2)
			}	// end else if (buffer_index_==1)
			else if (buffer_index_ == 2) {	//look for ack

				if (msg[i] == MSG_CLASS_ACK)   // ACK or NAK message class
						{
					// Get message id from payload
					char* class_id = reinterpret_cast<char*>(msg[i + 4]);
					char* msg_id = reinterpret_cast<char*>(msg[i + 5]);

					// Add function which takes class_id and msg_id and returns name of corresponding message

					if (msg[i + 1] == MSG_ID_ACK_ACK) // ACK Message
							{
						//std::cout << "Receiver Acknowledged Message " << std::endl;
						//printf("0x%.2X ", (unsigned)class_id);
						//std::cout << " ";
						//printf("0x%.2X ", (unsigned)msg_id);
						//std::cout << endl;

					}

					else if (msg[i + 1] == MSG_ID_ACK_NAK)    // NAK Message
							{
						//std::cout << "Receiver Did Not Acknowledged Message " << std::endl;
						//printf("0x%.2X ", (unsigned)class_id);
						//std::cout << " ";
						//printf("0x%.2X ", (unsigned)msg_id);
						//std::cout << endl;
					}

					buffer_index_ = 0;
					bytes_remaining_ = 0;
					//readingACK = false;			//? Why is readingACK = false in if & else statement? - CC
				} else {
					data_buffer_[buffer_index_++] = msg[i];
					//readingACK = false;
				}
			} else if (buffer_index_ == 3) {
				// msg[i] and msg[i-1] define message ID
				data_buffer_[buffer_index_++] = msg[i];
				// length of header is in byte 4

				//printHex(reinterpret_cast < char * > (data_buffer_),4);

				msgID = ((data_buffer_[buffer_index_ - 2]) << 8)
						+ data_buffer_[buffer_index_ - 1];
				//cout << "msgID = " << msgID << endl;
			} else if (buffer_index_ == 5) {
				// add byte to buffer
				data_buffer_[buffer_index_++] = msg[i];
				// length of message (payload + 2 byte check sum )
				bytes_remaining_ = ((data_buffer_[buffer_index_ - 1]) << 8)
						+ data_buffer_[buffer_index_ - 2] + 2;

				//cout << "bytes_remaining_ = " << bytes_remaining_ << endl;

				///cout << msgID << endl;
			} else if (buffer_index_ == 6) {	// set number of bytes
				data_buffer_[buffer_index_++] = msg[i];
				bytes_remaining_--;
			} else if (bytes_remaining_ == 1) {	// add last byte and parse
				data_buffer_[buffer_index_++] = msg[i];
				//std::cout << hex << (int)msg[i] << dec << std::endl;
				//cout << " msgID = " << msgID << std::endl;
				ParseLog(data_buffer_, msgID);
				// reset counters
				buffer_index_ = 0;
				bytes_remaining_ = 0;
				//cout << "Message Done." << std::endl;
			}  // end else if (bytes_remaining_==1)
			else {	// add data to buffer
				data_buffer_[buffer_index_++] = msg[i];
				bytes_remaining_--;
			}
		}	// end for
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error buffering incoming ublox data: " << e.what();
		log_error_(output.str());
	}
}

void Ublox::ParseLog(uint8_t *log, size_t logID) {
	try {
		uint16_t payload_length;
		uint8_t num_of_svs;
		uint8_t num_of_channels;

		switch (logID) {

		case AID_REQ: // Receiver outputs if accurate internally stored pos and time aren't available
			log_info_("AID-REQ message received by computer.");
			break;

		case CFG_PRT:
            ublox::CfgPrt cur_port_settings;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_port_settings, log, payload_length+HDR_CHKSM_LENGTH);
			//printHex((char*) &cur_port_settings, sizeof(cur_port_settings));
			if (port_settings_callback_)
				port_settings_callback_(cur_port_settings, read_timestamp_);
			break;

		case CFG_NAV5:
            ublox::CfgNav5 cur_nav5_settings;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav5_settings, log, payload_length+HDR_CHKSM_LENGTH);
			//printHex((char*) &cur_port_settings, sizeof(cur_port_settings));
			if (configure_navigation_parameters_callback_)
				configure_navigation_parameters_callback_(cur_nav5_settings, read_timestamp_);
			break;

		case NAV_STATUS:
            ublox::NavStatus cur_nav_status;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav_status, log, payload_length+HDR_CHKSM_LENGTH);
			if (nav_status_callback_)
				nav_status_callback_(cur_nav_status, read_timestamp_);
			break;

		case NAV_SOL:
            ublox::NavSol cur_nav_sol;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav_sol, log, payload_length+HDR_CHKSM_LENGTH);
			if (nav_sol_callback_)
				nav_sol_callback_(cur_nav_sol, read_timestamp_);
			break;

		case NAV_VELNED:
            ublox::NavVelNed cur_nav_vel_ned;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav_vel_ned, log, payload_length+HDR_CHKSM_LENGTH);
			if (nav_vel_ned_callback_)
				nav_vel_ned_callback_(cur_nav_vel_ned, read_timestamp_);
			break;

		case NAV_POSLLH:
            ublox::NavPosLLH cur_nav_position;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav_position, log, payload_length+HDR_CHKSM_LENGTH);
			if (nav_pos_llh_callback_)
				nav_pos_llh_callback_(cur_nav_position, read_timestamp_);
			break;

		case NAV_SVINFO:
            ublox::NavSVInfo cur_nav_svinfo;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			num_of_channels = (uint8_t) *(log+10);

			//std::cout << "NAV-SVINFO..." << std::endl;
			// print whole message
			//printHex((char*) log, payload_length+8);

			// Copy portion of NAV-INFO before repeated block (8 + header length)
			memcpy(&cur_nav_svinfo, log, 6+8);
			// Copy repeated block
			for(int index = 0; index < num_of_channels; index++) {
				memcpy(&cur_nav_svinfo.svinfo_reap[index], log+14+(index*12), 12);
			}
			// Copy Checksum
			memcpy(&cur_nav_svinfo.checksum, log+14+(num_of_channels*12), 2);

			// Print populated structure
			//printHex((char*) &cur_nav_svinfo, sizeof(cur_nav_svinfo));
			//std::cout << std::endl;

			if (nav_sv_info_callback_)
				nav_sv_info_callback_(cur_nav_svinfo, read_timestamp_);
			break;

		case NAV_GPSTIME:
            ublox::NavGPSTime cur_nav_gps_time;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav_gps_time, log, payload_length+HDR_CHKSM_LENGTH);
			if (nav_gps_time_callback_)
				nav_gps_time_callback_(cur_nav_gps_time, read_timestamp_);
			break;

		case NAV_UTCTIME:
            ublox::NavUTCTime cur_nav_utc_time;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav_utc_time, log, payload_length+HDR_CHKSM_LENGTH);
			if (nav_utc_time_callback_)
				nav_utc_time_callback_(cur_nav_utc_time, read_timestamp_);
			break;

		case NAV_DOP:
            ublox::NavDOP cur_nav_dop;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav_dop, log, payload_length+HDR_CHKSM_LENGTH);
			if (nav_dop_callback_)
				nav_dop_callback_(cur_nav_dop, read_timestamp_);
			break;

		case NAV_DGPS:
            ublox::NavDGPS cur_nav_dgps;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav_dgps, log, payload_length+HDR_CHKSM_LENGTH);
			if (nav_dgps_callback_)
				nav_dgps_callback_(cur_nav_dgps, read_timestamp_);
			break;

		case NAV_CLK:
            ublox::NavClock cur_nav_clock;
			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			memcpy(&cur_nav_clock, log, payload_length+HDR_CHKSM_LENGTH);
			if (nav_clock_callback_)
				nav_clock_callback_(cur_nav_clock, read_timestamp_);
			break;

		case AID_EPH:
            ublox::EphemSV cur_ephem_sv;

			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));

			//printHex((char*) &cur_ephem_sv, sizeof(cur_ephem_sv));
			memcpy(&cur_ephem_sv, log, payload_length+HDR_CHKSM_LENGTH);

			// If Ephemeris for SV is not present (payload_length is 8 bytes)
			if (payload_length == PAYLOAD_LENGTH_AID_EPH_NO_DATA)
			{
				//stringstream output;
				//output << "SV# " << (int) *(log+6) << "- no ephemeris data";
				//log_debug_(output.str());
			}
			// If Ephemeris for SV is present (payload_length is 104 bytes)
			else if (payload_length == PAYLOAD_LENGTH_AID_EPH_WITH_DATA)
			{
				//stringstream output;
				//output << "SV# " << (int) *(log+6) << "- has ephemeris data";
				//log_debug_(output.str());
			}
			else
			{
				log_error_("Error! AID-EPH log payload is not a valid length! (See ParseLog case AID_EPH)");
			}

			// make sure function pointer is set and call callback
			if (aid_eph_callback_)
				aid_eph_callback_(cur_ephem_sv, read_timestamp_);

			if (parsed_ephem_callback_) {
                ublox::ParsedEphemData parsed_ephem = Parse_aid_eph(cur_ephem_sv);
				parsed_ephem_callback_(parsed_ephem, read_timestamp_);
			}
			break;

		case AID_ALM:
            ublox::AlmSV cur_alm_sv;

			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));

			memcpy(&cur_alm_sv, log, payload_length+HDR_CHKSM_LENGTH);

			// If Almanac data for SV is not present (payload_length is 8 bytes)
			if (payload_length == 8)
			{
				//stringstream output;
				//output << "SV# " << (int) *(log+6) << "- no almanac data";
				//log_debug_(output.str());
			}

			// If Almanac data for SV is present (payload_length is 40 bytes)
			else if (payload_length == 40)
			{
				//stringstream output;
				//output << "SV# " << (int) *(log+6) << "- has almanac data";
				//log_debug_(output.str());
			}
			else
			{
				log_error_("Error! AID-ALM log payload is not 8 or 40 bytes long! (See ParseLog case AID_ALM)");
			}

			// make sure function pointer is set and call callback
			if (aid_alm_callback_)
				aid_alm_callback_(cur_alm_sv, read_timestamp_);
			break;

		case AID_HUI:
            ublox::AidHui cur_aid_hui;

			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));

			memcpy(&cur_aid_hui, log, payload_length+HDR_CHKSM_LENGTH);

			//log_debug_("Received AID-HUI Message.");

			//printHex((char*)log, sizeof(cur_aid_hui));

			// make sure function pointer is set and call callback
			if (aid_hui_callback_)
				aid_hui_callback_(cur_aid_hui, read_timestamp_);
			break;

		case AID_INI:
            ublox::AidIni cur_aid_ini;

			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));

			memcpy(&cur_aid_ini, log, payload_length+HDR_CHKSM_LENGTH);

			//printHex((char*) &cur_aid_ini, sizeof(cur_aid_ini));

			if (aid_ini_callback_)
				aid_ini_callback_(cur_aid_ini, read_timestamp_);

			break;

		case RXM_RAW:
		// NOTE: Needs to be checked/fixed
            ublox::RawMeas cur_raw_meas;

            payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4)); // payload_length = 8+24*numSV
			num_of_svs = (uint8_t) *(log+12);

			// Copy portion of RXM-SVSI before repeated block (8 + header length)
			memcpy(&cur_raw_meas, log, 6+8);
			// Copy repeated block
			for (uint8_t index = 0; index < num_of_svs; index++) {
				memcpy(&cur_raw_meas.rawmeasreap[index],log+14+(index*24),24);
			}
			// Copy Checksum
			memcpy(&cur_raw_meas.checksum, log+14+(24*num_of_svs), 2);

            if (rxm_raw_callback_)
                rxm_raw_callback_(cur_raw_meas, read_timestamp_);
			break;

        case RXM_SFRB:
            ublox::SubframeData cur_subframe;

            payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
            memcpy(&cur_subframe, log, payload_length+HDR_CHKSM_LENGTH);

            if (rxm_subframe_callback_)
                rxm_subframe_callback_(cur_subframe, read_timestamp_);
            break;

        case RXM_SVSI:
			// NOTE: needs to be checked!!
            ublox::SVStatus cur_sv_status;

			payload_length = (((uint16_t) *(log+5)) << 8) + ((uint16_t) *(log+4));
			num_of_svs = (uint8_t) *(log+13);

			/*
			std::cout << "number of svs following: "<<(double) num_of_svs << endl;
			std::cout << "payload length: "<<(double) payload_length << endl;
			printHex((char*) &payload_length,2);
			// print whole message
			std::cout << "RXM-SVSI..." << std::endl;
			printHex((char*) log, payload_length+8);
			std::cout <<std::endl;
			*/

			// Copy portion of RXM-SVSI before repeated block (8 + header length)
			memcpy(&cur_sv_status, log, 6 + 8);
			// Copy repeated block
			for (uint8_t index = 0; index < num_of_svs; index++) {
				memcpy(&cur_sv_status.svstatusreap[index],log+14+(index*6),6);
			}
			// Copy Checksum
			memcpy(&cur_sv_status.checksum, log+14+(6*num_of_svs), 2);

			/*
			// Print populated structure
			printHex((char*) &cur_sv_status, sizeof(cur_sv_status));
			std::cout << std::endl;
			*/

			if (rxm_svsi_callback_)
				rxm_svsi_callback_(cur_sv_status, read_timestamp_);
			break;

		} // end switch (logID)
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error parsing ublox log: " << e.what();
		log_error_(output.str());
	}
} // end ParseLog()

void Ublox::calculateCheckSum(uint8_t* in, size_t length, uint8_t* out) {

	try {
		uint8_t a = 0;
		uint8_t b = 0;

		for (uint8_t i = 0; i < length; i++) {

			a = a + in[i];
			b = b + a;

		}

		out[0] = (a & 0xFF);
		out[1] = (b & 0xFF);
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error calculating ublox checksum: " << e.what();
		log_error_(output.str());
	}
}

ParsedEphemData Ublox::Parse_aid_eph(EphemSV ubx_eph) {
    ublox::ParsedEphemData eph_data;
	try {
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

		//SVID
		eph_data.prn = (uint8_t) ubx_eph.svprn;

		//T_GD
		eph_data.tgd = ((char) ubx_eph.SF[0].W[4].byte[0]) * pow(2.0,-31);

		//t_oc
		union_unsigned_short.c[0] = ubx_eph.SF[0].W[5].byte[0];
		union_unsigned_short.c[1] = ubx_eph.SF[0].W[5].byte[1];
		eph_data.toc = ((int) union_unsigned_short.s) * pow(2.0,4);

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
	
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error parsing ephemeris data: " << e.what();
		log_error_(output.str());
	}
	return (eph_data);

};
