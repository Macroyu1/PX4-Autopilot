#include "servo_ctrl.hpp"
#include "uart.h"
#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>


ServoCtrl::ServoCtrl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
	{

	}
ServoCtrl::~ServoCtrl()
{

}

bool
ServoCtrl::init()
{
	return true;
}

void
ServoCtrl::Run()
{
	int succe;

	int uart_read = uart_init((char*)"/dev/ttyS6");
        if(false == uart_read)succe = -1;
        if(false == set_uart_baudrate(uart_read,115200)){
	printf("12%f",(double)succe);
     //   printf("[YCM]set_uart_baudrate is failed\n");

        }


}

void servo_control()
{
	char con0_write[16] =ser_ver.ser0;//1号舵机，P和T之间为驱动PWM值
        char con0_write1[10] ="#000PRAD!";//读取对应舵机的角度

	char con1_write[16] =ser_ver.ser1;
        char con1_write1[10] ="#001PRAD!";

	char con2_write[16] =ser_ver.ser2;
        char con2_write1[10] ="#002PRAD!";

	char con3_write[16] =ser_ver.ser3;
        char con3_write1[10] ="#003PRAD!";

	char con4_write[16] =ser_ver.ser4;
        char con4_write1[10] ="#004PRAD!";

	char con5_write[16] =ser_ver.ser5;
        char con5_write1[10] ="#005PRAD!";
}


int ServoCtrl::task_spawn(int argc,char *argv[])
{
	ServoCtrl *instance = new ServoCtrl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("servo control failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
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
extern "C" __EXPORT int servo_ctrl_main(int argc, char *argv[]);

int servo_ctrl_main(int argc, char *argv[])
{
	return ServoCtrl::main(argc, argv);
}




























// extern "C" __EXPORT int act_opt_main(int argc, char *argv[]);

// int act_opt_main(int argc, char **argv)
// {
//    int key_sub_fd = orb_subscribe(ORB_ID(OptimResult));
//     orb_set_interval(key_sub_fd, 200); // limit the update rate to 200ms

//     px4_pollfd_struct_t fds[1];
//     fds[0].fd = key_sub_fd, fds[0].events = POLLIN;

//     int error_counter = 0;

//     while(true)
//     {
//         int poll_ret = px4_poll(fds, 1, 1000);

//         if (poll_ret == 0)
//         {
//             PX4_ERR("Got no data within a second");
//         }

//         else if (poll_ret < 0)
//         {
//             if (error_counter < 10 || error_counter % 50 == 0)
//             {
//                 PX4_ERR("ERROR return value from poll(): %d", poll_ret);
//             }

//             error_counter++;
//         }

//         else
//         {
//             if (fds[0].revents & POLLIN)
//             {
//                 OptimResult_s input {};
//                 orb_copy(ORB_ID(OptimResult), key_sub_fd, &input);
//                 PX4_INFO("Recieved Char : %f", (double)input.opt_result[2]);
//              }
//         }
//     }
//     return 0;
// }
