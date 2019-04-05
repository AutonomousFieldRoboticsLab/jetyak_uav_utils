#ifndef BSC_COMMON_PID_
#define BSC_COMMON_PID_
#include <list>
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
	void updateParams(double kp, double ki, double kd);
	void reset();
	double get_signal();
};
}	// namespace bsc_common

#endif
