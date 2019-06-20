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
 * This class provides an interface for a PID controller
 * 
 * Author: Brennan Cain
 */
#ifndef BSC_COMMON_PID_
#define BSC_COMMON_PID_
#include <list>
#include <iostream>
namespace bsc_common
{
class PID
{
private:
	double kp_, ki_, kd_, last_error_, integral_, last_time_, signal_, last_d_;
	std::list<double> past_integral_contributions;
	int integral_frame_;
	bool use_int_frame_;

public:
	PID();
	PID(double kp, double ki, double kd, int integral_frame = 50);
	void update(double error, double utime);
	void update(double error, double utime, double vel);
	void updateParams(double kp, double ki, double kd);
	void reset();
	double get_signal();
};
} // namespace bsc_common

#endif
