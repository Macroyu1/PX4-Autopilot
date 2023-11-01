#include "servo_ctrl.hpp"
#include "uart.h"
#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <string.h>
using namespace std;

extern "C" __EXPORT int servo_ctrl_main(int argc, char *argv[]);

ServoCtrl::ServoCtrl() :
	ModuleParams(nullptr)
	{

	}
ServoCtrl::~ServoCtrl()
{
	// perf_free(_loop_perf);
}

bool
ServoCtrl::init()
{
	return true;
}

void
ServoCtrl::run()
{
	/*定义六个舵机的控制角度与读取角度信息命令*/
	serial_fd = open("/dev/ttyS6", O_RDWR | O_NOCTTY | O_NONBLOCK);
	set_uart_baudrate(serial_fd,115200);
	printf("uart init is successful\n");
	/*
	 * TELEM1 : /dev/ttyS1
	 * TELEM2 : /dev/ttyS2
	 * GPS    : /dev/ttyS3
	 * NSH    : /dev/ttyS5
	 * SERIAL4: /dev/ttyS6
	 * N/A    : /dev/ttyS4
	 * IO DEBUG (RX only):/dev/ttyS0
	 */
	while(1)
	{
		servo_control();
		servo_inquire();
		/* 发布 servos_a 数据到 servos_angel 话题 */
	}
}

void ServoCtrl::servo_control()
{
	int pwm[8]={};
	if(servo_ctrl_s.update(&servos_s)){
		// servo_ctrl_s.copy(&servos);
		for (int i = 0; i < 6; i++)
		{
		    servos[i] = servos_s.control[i];

		}
	}
	for (int i = 0; i < 6; i++)
	{
		pwm[i] = servos[i]*500 + 1500;
		//#000P1500T1000!
		// pwm[i] = 1500;
		char const *head = "#00";char const*p  = "P";char const*tail = "T1000!";
		char *buf = new char[strlen(head) + sizeof(i) /
			+ strlen(p) + sizeof(pwm[i]) +strlen(tail) + 1];
		sprintf(buf,"%s%d%s%d%s",head,i,p,pwm[i],tail);
		write(serial_fd,buf,16);
		usleep(500);
		// PX4_INFO("%s\n",buf);
		delete(buf);
	}
		// write(serial_fd,"#000P2000T1000!",16);
		// write(serial_fd,"#001P1000T1000!",16);
		// write(serial_fd,"#002P2000T1000!",16);
		// write(serial_fd,"#003P2000T1000!",16);
		// write(serial_fd,"#004P1500T1000!",16);
		// write(serial_fd,"#005P1000T1000!",16);
		// write(serial_fd,"#001P!",5);
}

void ServoCtrl::servo_inquire()
{
	char data = '0';
	char buffer[30] = "0";
	int ser;double ser_deg[6];
	//#000PRAD!
	char const *head = "#00";char const*tail = "PRAD!";
	for (int i = 0; i < 6; i++)
	{
		char *buf = new char[strlen(head) + strlen(tail) + 2];
		sprintf(buf,"%s%d%s",head,i,tail);
		// PX4_INFO("%s\n",buf);
		write(serial_fd,buf,9);
		delete(buf);
		usleep(500);
	}

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			read(serial_fd,&data,1);
			buffer[j] = data;
			data = '0';
			usleep(500);
		}
		if((int)*buffer == 35){//#
			ser = atoi(buffer+5);ser = ser > 1000 ? (ser < 2000) ? ser : 1500 : 1500;
			ser_deg[buffer[3]-48] = ((ser-1500)*90.f/500.f);
			// PX4_INFO("%f\n",servos_a.angel[buffer[3]-48]);
		}
	}
	for (int i = 0; i < 5; i++)
	{
	    servos_a.angel[i] = ser_deg[i];
	}

	servos_angel_pub.publish(servos_a);
}

int ServoCtrl::task_spawn(int argc,char *argv[])
{
	_task_id = px4_task_spawn_cmd("ServoCtrl",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      3150,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return 0;
	}

	return 0;
}

ServoCtrl *ServoCtrl::instantiate(int argc, char *argv[])
{
	ServoCtrl *instance=new ServoCtrl();
	if(instance==nullptr)
	{
		PX4_ERR("alloc failed");
	}
	return instance;
}

int ServoCtrl::print_status()
{
	PX4_INFO("Running");
	return 0;
}

int ServoCtrl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ServoCtrl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	return 0;
}

/**
 * Servo Control app start / stop handling function
 */
int servo_ctrl_main(int argc, char *argv[])
{
	return ServoCtrl::main(argc, argv);
}


