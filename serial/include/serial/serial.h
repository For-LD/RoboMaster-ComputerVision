#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <mutex>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <string.h>
#include <stdexcept>
#include <exception>
#include <unistd.h>
#include <fstream>
#include <memory>
#include <memory.h>
#include <iomanip>

#undef EOF

struct Test_send
{
	//uint8_t name;
	//int id;
	//int distance;
	float yaw_angle;
	float pitch_angle;
	//uint16_t detail;
};

struct Test_receive
{
	int ctrl_mode;
	int task_mode;

};


class Serial {

private:

	struct Test_send_frame
	{
		uint8_t sof;
		//uint8_t name;
		//int id;
		//int distance;
		float pitch_angle;
		float yaw_angle;
		//uint16_t detail;
		uint8_t eof;

	}_test_send_frame;

	Test_receive _test_receive;

	int _serialFd;

	enum
	{
		JetsonCommSOF = (uint8_t)0x66,
		JetsonCommEOF = (uint8_t)0x88
	};

public:

	int openPort();
	int closePort();
	int send(const Test_send& test_send);
	Test_receive receive();
	//Serial::Test_receive get_receive();
	Test_send_frame pack(const Test_send& ctrl);
	static uint16_t TransShort(uint8_t low, uint8_t high);
	static float TransFloat(uint8_t f1, uint8_t f2, uint8_t f3, uint8_t f4);
};
