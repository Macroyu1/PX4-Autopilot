/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2023-08-05 10:17:44
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2023-12-26 15:48:32
 * @FilePath: /PX4-Autopilot/src/modules/mavlink/streams/ACTUATOR_OPTIM.hpp
 * @Description:
 *
 * Copyright (c) 2023 by HongYu Fu, All Rights Reserved.
 */
// /****************************************************************************
//  *
//  *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  * 3. Neither the name PX4 nor the names of its contributors may be
//  *    used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/

// #ifndef ACTUATOR_OPTIM_HPP
// #define ACTUATOR_OPTIM_HPP

// #include <uORB/topics/ActuatorOptim.h>

// class MavlinkStreamActuatorOptim : public MavlinkStream
// {
// public:
// 	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamActuatorOptim(mavlink); }

// 	static constexpr const char *get_name_static() { return "ACTUATOR_OPTIM"; }

// 	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ACTOPT_LEN; }

// 	const char *get_name() const override { return get_name_static(); }
// 	uint16_t get_id() override { return get_id_static(); }

// 	unsigned get_size() override
// 	{
// 		return _act_optim_sub.advertised() ? (MAVLINK_MSG_ID_ACTOPT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
// 	}

// private:
// 	explicit MavlinkStreamActuatorOptim(Mavlink *mavlink) : MavlinkStream(mavlink){}

// 	uORB::Subscription _act_optim_sub{ORB_ID(ActuatorOptim)};

// 	bool send() override
// 	{
// 		ActuatorOptim_s act_optim;

// 		if (_act_optim_sub.update(&act_optim)) {
// 			mavlink_actopt_t msg{};

// 			for (unsigned i = 0; i < 12; i++) {
// 				msg.controls[i] = act_optim.controls[i];
// 			}

// 			mavlink_msg_actopt_send_struct(_mavlink->get_channel(), &msg);

// 			return true;
// 		}

// 		return false;
// 	}
// };

// #endif // ACTUATOR_OPTIM_HPP
