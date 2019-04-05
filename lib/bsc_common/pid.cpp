#include <iostream>
#include "include/pid.h"
namespace bsc_common
{
PID::PID() : PID(0.0, 0.0, 0.0){};

PID::PID(double kp, double ki, double kd, int integral_frame) : past_integral_contributions()	// instantiate the list
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
	use_int_frame_ = integral_frame >= 0;	// true if integral frame valid size
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
	else if (last_time_ != 0)	// if not first time
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

			if (use_int_frame_)	// allows
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
	last_error_ = 0;
	last_time_ = 0;
	integral_ = 0;
	last_d_ = 0;
	if (!past_integral_contributions.empty())
		past_integral_contributions.clear();
}
}	// namespace bsc_common
