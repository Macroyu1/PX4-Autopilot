#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/servos_angel.h>

class ServoCtrl : public ModuleBase<ServoCtrl>, public ModuleParams
{
	public:
		ServoCtrl();

		virtual ~ServoCtrl();

		/** @see ModuleBase */
		static int task_spawn(int argc, char *argv[]);

		/** @see ModuleBase */
		static int custom_command(int argc, char *argv[]);

		/** @see ModuleBase */
		static int print_usage(const char *reason = nullptr);

		/** @see ModuleBase::print_status() */
		int print_status() override;

		void run() override;

		bool init();

		static ServoCtrl *instantiate(int argc, char *argv[]);

		void servo_control();
		void servo_inquire();
		//Inputs
		uORB::Subscription servo_ctrl_s{ORB_ID(actuator_servos)};
		//Outputs
		uORB::Publication<servos_angel_s> servos_angel_pub{ORB_ID(servos_angel)};


		actuator_servos_s 	servos_s {};
		// servos_angel_s servos_a {};
		servos_angel_s 		servos_a {};
		float servos[8];
		int serial_fd;
		int count = 0;
	private:


		// perf_counter_t	_loop_perf;			/**< loop duration performance counter */

};





































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
