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
 * This class provides an interface for an ENU reference
 * Converts a gps signal (longitude, latitude, height) to a local cartesian ENU system
 *	
 * Use setENUorigin(lat, lon, height) to set the local ENU coordinate system
 * Use geo2enu(lat, lon, height) to get position in the local ENU system
 * 
 * Author: Michail Kalaitzakis
 * Ported by Brennan Cain from python to C++ 
 */
#ifndef BSC_COMMON_GPS_ENU_
#define BSC_COMMON_GPS_ENU_
#include <list>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace bsc_common
{
class GPS_ENU
{
private:
	// Geodetic System WGS 84 Axes
	double a = 6378137;
	double b = 6356752.314245;
	double a2 = a * a;
	double b2 = b * b;
	double e2 = 1 - (b2 / a2);

	// local ENU
	double latZero = 0;
	double lonZero = 0;
	double altZero = 0;
	double xZero = 0;
	double yZero = 0;
	double zZero = 0;
	Eigen::Matrix<double, 3, 1> oZero = Eigen::Matrix<double, 3, 1>::Identity();
	Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity();

public:
	GPS_ENU();
	void setENUOrigin(double lat, double lon, double alt);
	Eigen::Matrix<double, 3, 1> geo2ecef(double lat, double lon, double alt);
	Eigen::Matrix<double, 3, 1> ecef2enu(double x, double y, double z);
	Eigen::Matrix<double, 3, 1> geo2enu(double lat, double lon, double alt);
};
} // namespace bsc_common

#endif
