/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file infrared_tof.cpp
 */

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>

#include <uORB/topics/infrared_tof.h>

#define ULANDING_MIN_DISTANCE		0.1f
#define ULANDING_MAX_DISTANCE		32.0f


#define TOF_DEFAULT_PORT		"/dev/ttyS6"	// Serial4 on Pixhawk
#define BUFF_LEN 		45

//根据用户平台自定义字长
typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned long U32;
typedef char S8;
typedef short S16;
typedef long S32;


unsigned int crc_ta_8[256] = { /* CRC 字节余式表 */
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};


extern "C" __EXPORT int infrared_tof_main(int argc, char *argv[]);

class ToF : public device::CDev
{
public:
	ToF(const char *port = TOF_DEFAULT_PORT);
	virtual ~ToF();

	virtual int 			init();

	int				start();
	
	unsigned int crc_cal_by_byte(unsigned char* ptr, int len);

private:
	bool				_task_should_exit;
	int 				_task_handle;
	char 				_port[20];
	int				_class_instance;
	int				_orb_class_instance;
	orb_advert_t			_distance_sensor_topic;
	
	U8 _buf[BUFF_LEN];
	static void task_main_trampoline(int argc, char *argv[]);
	void task_main();

	bool decode(U8 *packet);
	bool read_and_parse(U8 *buf,int len, U8 data_buff[]);

};

namespace tof
{
ToF	*g_dev;
}

ToF::ToF(const char *port) :
	CDev("ToF", RANGE_FINDER1_DEVICE_PATH),
	_task_should_exit(false),
	_task_handle(-1),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr)
{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	// disable debug() calls
	_debug_enabled = false;

	memset(&_buf[0], 0, sizeof(_buf));
}

ToF::~ToF()
{

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	if (_task_handle != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_task_handle);
				break;
			}
		} while (_task_handle != -1);
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	orb_unadvertise(_distance_sensor_topic);
}

/**
  * @brief  计算数据包CRC16 CCITT校验码
  * @param  
  *			1、ptr：数据包首地址
  *			2、len：数据包要参与计算的数据字节数
  * @note   
  * @retval 返回数据包的 CRC16 值
  */
unsigned int 
ToF::crc_cal_by_byte(unsigned char* ptr, int len)
{
	unsigned short crc = 0xffff;

	while (len-- != 0)
	{
		unsigned int high = (unsigned int)(crc / 256);

		crc <<= 8;
		crc ^= crc_ta_8[high^*ptr];
		ptr++;
	}
	return crc;
}

/*
 * Test whether we got the right packet
 */
bool
ToF::decode(U8 *packet)
{
	U16 crc16_cal = 0x0000;
	U16 crc16_packet = 0x0000;

	bool ret = false;

	crc16_packet = packet[11]<<8;
	crc16_packet |= packet[12];
	crc16_cal = crc_cal_by_byte(&packet[0],11);

	if(crc16_cal == crc16_packet)
	{
		ret = true;
	}

	return ret;
}

/**
  * @brief  传感器返回的数据包解析
  * @param  
  *			1、packet：数据包首地址，长度为固定为10个字节的数组
  *         2、len 
  *			3、packet[]：存储解析后的数据缓冲区
  * @note   先根据返回值判断是否解析成功，才使用data_buff[]的数据
  *			注意：确保packet和data_buff的内存区域不能重叠
  * @retval 解析成功返回 true，失败返回 false
  */
bool 
ToF::read_and_parse(U8 *buf,int len, U8 packet[])
{
	bool ret = false;
	static uint8_t p_flag = 0;
	static int process_counter = 0;

	for (unsigned i = 0; i < len; i++) {
		switch(process_counter)
		{
			case 0:
				if(0x0A == buf[i])
					process_counter ++;
				break;
			case 1:
				if(0x0D == buf[i]){
					process_counter ++; 
				}else{
					process_counter = 0;
				}
				break;
			case 2:
				packet[p_flag++] = buf[i];
				if(0x0d == p_flag){
					p_flag = 0;
					process_counter = 0;
					if(decode(packet))
					{
					//	printf(" Get the packet ! \n");
						ret = true;
					}	
				}
				break;
			default:
				process_counter = 0;
				break;	
		}
	}
	return ret ;
}

int
ToF::init()
{
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) {
			PX4_WARN("cdev init failed");
			break;
		}

		int fd = px4_open(RANGE_FINDER1_DEVICE_PATH, 0);

		if (fd < 0) {
			PX4_WARN("failed to open range finder device");
			ret = 1;
			break;
		}

		px4_close(fd);

		/* open fd */
		fd = px4_open(_port, O_RDWR | O_NOCTTY);

		if (fd < 0) {
			PX4_WARN("failed to open serial device");
			ret = 1;
			break;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag	 &= ~(CSTOPB | PARENB);

		unsigned speed = B115200;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d ISPD", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d OSPD\n", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
			PX4_WARN("ERR baud %d ATTR", termios_state);
			ret = 1;
			break;
		}

		px4_close(fd);

		_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

		struct infrared_tof_s ds_report={};
		ds_report.timestamp = hrt_absolute_time();
		// make evident that this range sample is invalid
		ds_report.distance = -1;
		ds_report.magnitude = 0;
		ds_report.magnitude_exp = 0;
		ds_report.ambient_adc = 0;
		ds_report.precision = 0;

		_distance_sensor_topic = orb_advertise(ORB_ID(infrared_tof), &ds_report);

		if (_distance_sensor_topic == nullptr) {
			DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
			ret = 1;
			break;
		}

	} while (0);

	return ret;
}

void
ToF::task_main_trampoline(int argc, char *argv[])
{
	tof::g_dev->task_main();
}

int
ToF::start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("infrared_tof",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX - 30,
					  1200,
					  (px4_main_t)&ToF::task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}


void
ToF::task_main()
{

	int fd = px4_open(_port, O_RDWR | O_NOCTTY);

	// we poll on data from the serial port
	px4_pollfd_struct_t fds[1];
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	// read buffer, one measurement consists of three bytes
	U8 buf[BUFF_LEN];
	static U8 packet[13]={0};

	// Stop mode
	// unsigned char comm0[10] ={0x0A, 0x30, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x6F};
	// px4_write(fd,&comm0[0],10);

	// // Device Info
	// unsigned char comm1[10] = {0x0A, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x3C};

	// // Auto scan
	// unsigned char comm2[10] = {0x0A, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x99, 0x2C};

	// // Single measure
	// unsigned char comm3[10] = {0x0A, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAE, 0x57};


	
	// px4_write(fd,&comm1[0],10);
	// px4_write(fd,&comm1[0],10);
	// px4_write(fd,&comm2[0],10);
	// px4_write(fd,&comm3[0],10);


	// //Continuous mode
	unsigned char comm[10] = {0x0A, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x72};
	
	px4_write(fd,&comm[0],10);

	printf("Starting logging \n");

	// FILE *log = NULL;
	// log = fopen("/fs/microsd/AAA.txt","a+");
	while (!_task_should_exit) {
		
		// wait for up to 50ms for data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		// timed out
		if (pret == 0) {
			continue;
		}

		if (pret < 0) {
			PX4_DEBUG("ToF serial port poll error");
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			memset(&buf[0], 0, sizeof(buf));
			int len = px4_read(fd, &buf[0], sizeof(buf));
			//printf("length is %d \n",len);

			// for(int k =0;k<len;k++)
			// {
			// 	fprintf(log,"%x",buf[k]);
			// }

			if (len <= 0) {
				PX4_DEBUG("error reading tof");
			}

			if(read_and_parse(buf,len,packet))
			{
				struct infrared_tof_s dbuf;
				dbuf.timestamp = hrt_absolute_time();
				dbuf.distance = (packet[3] << 8)|packet[4];
				dbuf.magnitude = (packet[5] << 8)|packet[6];
				dbuf.magnitude_exp = packet[7];
				dbuf.ambient_adc = packet[8];
				dbuf.precision = (packet[9] << 8)|packet[10];
				//printf("The distance is %d \n",dbuf.distance);
				
				// publish it
				orb_publish(ORB_ID(infrared_tof), _distance_sensor_topic, &dbuf);
			}

		}
	}

	px4_close(fd);
}

int infrared_tof_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (tof::g_dev != nullptr) {
			PX4_WARN("driver already started");
			return 0;
		}

		if (argc > 2) {
			tof::g_dev = new ToF(argv[2]);

		} else {
			tof::g_dev = new ToF(TOF_DEFAULT_PORT);
		}

		if (tof::g_dev == nullptr) {
			PX4_ERR("failed to create instance of ToF");
			return 1;
		}

		if (PX4_OK != tof::g_dev->init()) {
			delete tof::g_dev;
			tof::g_dev = nullptr;
			return 1;
		}

		if (OK != tof::g_dev->start()) {
			delete tof::g_dev;
			tof::g_dev = nullptr;
			return 1;
		}

		return 0;
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		if (tof::g_dev != nullptr) {
			delete tof::g_dev;
			tof::g_dev = nullptr;

		} else {
			PX4_WARN("driver not running");
		}

		return 0;
	}

	if (!strcmp(argv[1], "info")) {
		PX4_INFO("Infrared ToF Range Finder ");
		PX4_INFO("min distance %.2f m", (double)ULANDING_MIN_DISTANCE);
		PX4_INFO("max distance %.2f m", (double)ULANDING_MAX_DISTANCE);
		PX4_INFO("auto customed update rate up to 54 Hz");
		return 0;
	}

	PX4_WARN("unrecognized arguments, try: start [device_name], stop, info ");
	return 1;
}
