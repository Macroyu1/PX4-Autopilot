/*
 * @Author: Macroyu1 1628343763@qq.com
 * @Date: 2023-08-02 09:17:32
 * @LastEditors: Macroyu1 1628343763@qq.com
 * @LastEditTime: 2024-01-10 10:18:02
 * @FilePath: /PX4-Autopilot/src/lib/ladrc/ladrc.cpp
 * @Description:
 *
 * Copyright (c) 2024 by HongYu Fu, All Rights Reserved.
 */
/**
 * @file adrc.c
 *
 * Implementation of generic ADRC controller.
 *
 * @author sunyi
 */
#include "ladrc.hpp"
#include <math.h>
#include <px4_platform_common/defines.h>

LADRC::LADRC(const float wc_in,const float b0_in,const float c20_in)
{
		this->wc = wc_in;
		this->b0 = b0_in;
		this->z1 = 0;
		this->z2 = 0;
		this->z3 = 0;
		this->u  = 0;
		this->u0 = 0;
		this->w0 = this->wc * c20_in;
	}
