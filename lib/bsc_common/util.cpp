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
 * This file implements utility functions
 * Author: Brennan Cain
 */

#include "include/util.h"
void bsc_common::util::rpy_from_quat(const geometry_msgs::Quaternion &orientation, geometry_msgs::Vector3 *state)
{
	// Get message as quaternion, then get roll, pitch, yaw
	tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
	tf::Matrix3x3 m(q);
	double t_r, t_p, t_y;
	m.getRPY(t_r, t_p, t_y);
	state->x = t_r;
	state->y = t_p;
	state->z = t_y;
	if (state->x > C_PI)
	{
		state->x = state->x - 2 * C_PI;
	}
	if (state->y > C_PI)
	{
		state->y = state->y - 2 * C_PI;
	}
	if (state->z > C_PI)
	{
		state->z = state->z - 2 * C_PI;
	}
}

double bsc_common::util::yaw_from_quat(const geometry_msgs::Quaternion &orientation)
{
	geometry_msgs::Vector3 *state = new geometry_msgs::Vector3();

	bsc_common::util::rpy_from_quat(orientation, state);

	double yaw = state->z;
	if (yaw > bsc_common::util::C_PI)
	{
		yaw = yaw - 2 * bsc_common::util::C_PI;
	}
	return yaw;
}

template <typename T>
T bsc_common::util::clip(T x, T low, T high)
{
	return std::max(std::min(high, x), low);
}

void bsc_common::util::rotate_vector(double x, double y, double theta, double &xp, double &yp)
{
	xp = (x * std::cos(theta) - y * std::sin(theta));
	yp = (x * std::sin(theta) + y * std::cos(theta));
}

/* rotation_matrix
	 *
	 * @param theta angle to rotate
	 */
Eigen::Matrix2d bsc_common::util::rotation_matrix(double theta)
{
	Eigen::Matrix2d rot;
	rot << std::cos(theta), -std::sin(theta),
			std::sin(theta), std::cos(theta);
	return rot;
}

void bsc_common::util::inverse_pose(const geometry_msgs::Pose &in, geometry_msgs::Pose &out)
{
	tf2::Transform trans;
	tf2::fromMsg(in, trans);
	geometry_msgs::Pose inverse_pose;
	const tf2::Transform inverse_trans = trans.inverse();
	tf2::toMsg(inverse_trans, out);
}
double bsc_common::util::ang_dist(double start, double stop, bool rad)
{
	double pi = rad ? C_PI : 180;

	while (start < -pi)
		start += 2 * pi;
	while (stop < -pi)
		stop += 2 * pi;
	while (start >= pi)
		start -= 2 * pi;
	while (stop >= pi)
		stop -= 2 * pi;

	int d1 = stop - start;
	if (d1 >= pi)
		d1 = d1 - 2 * pi;
	if (d1 < -pi)
		d1 = d1 + 2 * pi;
	return d1;
}

double bsc_common::util::latlondist(double lat1, double lon1, double lat2, double lon2)
{
	double R = 6378137; // Radius of earth in m

	double rLat1 = lat1 * C_PI / 180;
	double rLat2 = lat2 * C_PI / 180;
	double rLon1 = lon1 * C_PI / 180;
	double rLon2 = lon2 * C_PI / 180;

	double dLat = rLat2 - rLat1;
	double dLon = rLon2 - rLon1;
	// a = sin^2(dla/2)+cos(la1)cos(la2)sin^2(dlo/2)
	double a = pow(sin(dLat / 2), 2) + cos(rLat1) * cos(rLat2) * pow(sin(dLon / 2), 2);
	return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

bool bsc_common::util::insensitiveEqual(std::string &str1, std::string &str2)
{
	return ((str1.size() == str2.size()) && std::equal(str1.begin(), str1.end(), str2.begin(), [](char &c1, char &c2) {
						return (c1 == c2 || std::toupper(c1) == std::toupper(c2));
					}));
}

// x/(1+|x|)
double bsc_common::util::fastSigmoid(double x)
{
	return x / (1 + abs(x));
}
