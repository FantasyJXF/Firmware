/**
 * @file NMEA.cpp
 *
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctime>

#include "nmea.h"
#include "rtk.h"


#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define SWAP16(X)	((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))


GPSDriverNMEA::GPSDriverNMEA(Interface interface, GPSCallbackPtr callback, void *callback_user,
			   struct vehicle_gps_position_s *gps_position,
                             struct satellite_info_s *satellite_info):
    GPSHelper(callback, callback_user)
{
    _satellite_info = satellite_info;
    _gps_position = gps_position;
    _rx_buffer_bytes = 0;
    _got_pashr_pos_message = false;
    _parse_error = false;
    memset(_rx_buffer, 0, sizeof(_rx_buffer));
    _decode_state = NMEA_DECODE_UNINIT;
    _last_timestamp_time = 0;
	_parse_pos = 0;
}

GPSDriverNMEA::~GPSDriverNMEA()
{
}

int
GPSDriverNMEA::configure(unsigned &baudrate, OutputMode output_mode)
{
	baudrate = RTK_DEFAULT_BAUDRATE;
	setBaudrate(baudrate);
	return 0;
}

int
GPSDriverNMEA::parseChar(uint8_t b)
{
    int iRet = 0;
    switch (_decode_state){
        case NMEA_DECODE_UNINIT:
            if (b == '$') {
                _decode_state =NMEA_DECODE_GOT_SYNC1;
                _rx_buffer_bytes= 0;
                _rx_buffer[_rx_buffer_bytes++]= b;
            }
            break;
        case NMEA_DECODE_GOT_SYNC1:
            if (b == '$') {
                _decode_state =NMEA_DECODE_GOT_SYNC1;
                _rx_buffer_bytes= 0;
            } else if (b =='*') {
                _decode_state =NMEA_DECODE_GOT_NMEA;
            }
            if(_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
                _decode_state =NMEA_DECODE_UNINIT;
                _rx_buffer_bytes= 0;
            } else {
                _rx_buffer[_rx_buffer_bytes++]= b;
            }
            break;
        case NMEA_DECODE_GOT_NMEA:
            _rx_buffer[_rx_buffer_bytes++]= b;
            _decode_state =NMEA_DECODE_GOT_FIRST_CS_BYTE;
            break;
        case NMEA_DECODE_GOT_FIRST_CS_BYTE:
            _rx_buffer[_rx_buffer_bytes++]= b;
            uint8_t checksum =0;
            uint8_t *buffer =_rx_buffer + 1;
            uint8_t *bufend =_rx_buffer + _rx_buffer_bytes - 3;
            for (; buffer <bufend; buffer++) { checksum ^= *buffer; }
            if((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2))&&(HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer +_rx_buffer_bytes - 1))) {
                iRet =_rx_buffer_bytes;
            }
            _decode_state =NMEA_DECODE_UNINIT;
            _rx_buffer_bytes =0;
            break;
    }
    return iRet;
}

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverNMEA::receive(unsigned timeout)
{
	gps_abstime time_started = gps_absolute_time();

	while (true) {

		if (_parse_pos >= _read_total) {
			_parse_pos = 0;
			_read_total = read(buf, sizeof(buf), timeout);
		}
		if (_read_total<= 0) {
            usleep(20000);
		} else {
			int rr;
			/* pass received bytes to the packet decoder */
			for (;  _parse_pos < _read_total; _parse_pos++) {
                if ((rr = parseChar(buf[_parse_pos])) >0 ) {
					_rx_buffer[rr] = 0;
					PX4_INFO("handle:%s", _rx_buffer);
                    rr = handleMessage(rr);
					if (rr > 0) {
						_parse_pos ++;
						return rr;
					}
                }
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			return -1;
		}
	}
}

int
GPSDriverNMEA::handleMessage(int len)
{
    int num_comma = 0;
    for (int i = 0 ; i <len; i++) {
        if (_rx_buffer[i]== ',') { num_comma ++; }
    }
    char ns = 0, ew = 0;
    double lat = 0, lon = 0;
    char * endp;
    char *bufptr = (char *)(_rx_buffer + 7);
    if ((memcmp(_rx_buffer + 3, "GGA,", 3) == 0) && (num_comma == 14))
    {
        if (bufptr &&*(++bufptr) != ',') { strtod(bufptr, &endp); bufptr = endp;}
        if (bufptr&& *(++bufptr) != ',') { lat = strtod(bufptr, &endp); bufptr =endp; }
        if (bufptr&& *(++bufptr) != ',') { ns = *(bufptr++); }
        if (bufptr&& *(++bufptr) != ',') { lon = strtod(bufptr, &endp); bufptr =endp; }
        if (bufptr&& *(++bufptr) != ',') { ew = *(bufptr++); }
        if (bufptr&& *(++bufptr) != ',') { _gps_position->fix_type=1+strtol(bufptr, &endp, 10);bufptr = endp; }
        if (bufptr&& *(++bufptr) != ',') { _gps_position->satellites_used = strtol(bufptr, &endp, 10);bufptr = endp; }
        if (bufptr&& *(++bufptr) != ',') { _gps_position->hdop = strtod(bufptr, &endp); bufptr =endp; }
        if (bufptr&& *(++bufptr) != ',') { _gps_position->alt = strtod(bufptr, &endp) * 1000; bufptr =endp; }
		lat /= 100; lon /= 100;
        _gps_position->lat = lat;
        _gps_position->lon = lon;
		lat = (lat - _gps_position->lat) / 6 * 10 + _gps_position->lat;
		lon = (lon - _gps_position->lon) / 6 * 10 + _gps_position->lon;	
        if (ns == 'S') {lat= -lat;}
        if (ew == 'W') {lon= -lon;}
        _gps_position->lat = lat * 10000000;
        _gps_position->lon = lon * 10000000;
        _gps_position->timestamp = gps_absolute_time();
        _gps_position->timestamp_time_relative = 0;
        return 1;
    }
    return 0;
}

uint8_t
GPSDriverNMEA::HEXDIGIT_CHAR(uint8_t c)
{
    if (c < 10) return '0' + c;
    else return 'A' + c - 10;
}

