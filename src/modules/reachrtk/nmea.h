/**
 * @file nmea.h
 *
 */

#ifndef _NMEA_H 
#define _NMEA_H

#include "gps_helper.h"
#include "definitions.h"

#define NMEA_RECV_BUFFER_SIZE 255


typedef enum {
	NMEA_DECODE_UNINIT,
	NMEA_DECODE_GOT_SYNC1,
	NMEA_DECODE_GOT_NMEA,
	NMEA_DECODE_GOT_FIRST_CS_BYTE
} nmea_decode_state_t;

class GPSDriverNMEA : public GPSHelper
{
public:
	GPSDriverNMEA(Interface interface, GPSCallbackPtr callback, void *callback_user,
		     struct vehicle_gps_position_s *gps_position,
		     struct satellite_info_s *satellite_info);
	virtual ~GPSDriverNMEA();
	int receive(unsigned timeout);
	int configure(unsigned &baudrate, OutputMode output_mode);

private:
	int handleMessage(int len);
	int parseChar(uint8_t b);
	int32_t read_int();
	double read_float();
	char read_char();
    struct satellite_info_s *_satellite_info;
	struct vehicle_gps_position_s *_gps_position;
	uint64_t _last_timestamp_time;
	nmea_decode_state_t _decode_state;
	uint8_t _rx_buffer[NMEA_RECV_BUFFER_SIZE];
	uint16_t _rx_buffer_bytes;
	uint8_t buf[GPS_READ_BUFFER_SIZE];
	bool _got_pashr_pos_message;
	bool _parse_error;
	uint16_t _parse_pos, _read_total;
	uint8_t HEXDIGIT_CHAR(uint8_t c);
};

#endif 
