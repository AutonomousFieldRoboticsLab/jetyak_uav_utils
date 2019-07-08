/**
MIT License

Copyright (c) 2018 Brennan Cain and Michail Kalaitzakis (Unmanned Systems and Robotics Lab, University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 * This file implements a PID controller
 * 
 * Author: Brennan Cain
 */

#include "include/pid.h"
#include <iostream>
namespace bsc_common
{
PID::PID() : PID(0.0, 0.0, 0.0){};

PID::PID(double kp, double ki, double kd, int integral_frame) : past_integral_contributions() // instantiate the list
{
	kp_ = kp;
	kd_ = kd;
	ki_ = ki;
	last_error_ = 0;
	last_time_ = 0;
	integral_ = 0;
	signal_ = 0;
	last_d_ = 0;
	integral_frame_ = integral_frame;
	use_int_frame_ = integral_frame >= 0; // true if integral frame valid size
}

double PID::get_signal()
{
	return signal_;
}

void PID::update(double error, double utime)
{
	// Proportional
	signal_ = error * kp_;
	if (utime == 0)
	{
		std::cout << "PID NEVER RECEIVED TIMESTAMPED ERROR" << std::endl;
	}
	else if (last_time_ != 0) // if not first time
	{
		if (last_time_ == utime)
		{
			signal_ += integral_ * ki_ + last_d_;
		}
		else
		{
			double i, d;

			// get change in time
			double dt = utime - last_time_;

			// integral
			integral_ += error * dt;
			i = integral_ * ki_;

			// differential
			d = kd_ * (error - last_error_) / dt;

			last_d_ = d;

			signal_ += i + d;

			if (use_int_frame_) // allows
			{
				// Add current integral contribution to the list
				past_integral_contributions.push_back(error * dt);
				// If we have too many elements
				if (past_integral_contributions.size() > integral_frame_)
				{
					// remove the oldest and subtract it's contribution to the rolling sum
					integral_ -= past_integral_contributions.front();
					// remove it
					past_integral_contributions.pop_front();
				}
			}
		}
	}
	last_error_ = error;
	last_time_ = utime;
}

void PID::update(double error, double utime, double vel)
{
	// Proportional
	signal_ = error * kp_;
	if (utime == 0)
	{
		std::cout << "PID NEVER RECEIVED TIMESTAMPED ERROR" << std::endl;
	}
	else if (last_time_ != 0) // if not first time
	{
		if (last_time_ == utime)
		{
			signal_ += integral_ * ki_ + last_d_;
		}
		else
		{
			double i, d;

			// get change in time
			double dt = utime - last_time_;

			// integral
			integral_ += error * dt;
			i = integral_ * ki_;

			// differential
			d = kd_ * vel;

			last_d_ = d;

			signal_ += i + d;

			if (use_int_frame_) // allows
			{
				// Add current integral contribution to the list
				past_integral_contributions.push_back(error * dt);
				// If we have too many elements
				if (past_integral_contributions.size() > integral_frame_)
				{
					// remove the oldest and subtract it's contribution to the rolling sum
					integral_ -= past_integral_contributions.front();
					// remove it
					past_integral_contributions.pop_front();
				}
			}
		}
	}
	last_error_ = error;
	last_time_ = utime;
}

void PID::updateParams(double kp, double ki, double kd)
{
	kp_ = kp;
	kd_ = kd;
	ki_ = ki;
}

void PID::reset()
{
	std::cout << "Resetting" << std::endl;
	last_error_ = 0;
	last_time_ = 0;
	integral_ = 0;
	last_d_ = 0;
	if (!past_integral_contributions.empty())
		past_integral_contributions.clear();
}
} // namespace bsc_common
