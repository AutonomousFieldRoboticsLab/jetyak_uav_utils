#include "include/lqr.h"
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

namespace bsc_common
{
LQR::LQR(Eigen::Matrix<double, 4, 12> K)
{
	this->K = K;
}

LQR::LQR(std::string file)
{
	K = Eigen::Matrix<double, 4, 12>::Zero();
	std::ifstream infile;
	infile.open(file);
	std::string delim = " ";
	std::string token;
	while (!infile.eof())
	{
		std::string line;
		std::getline(infile, line);
		char pos = 0;
		int i = 0;
		int col, row;
		double val;
		std::cout << line << std::endl;
		while ((pos = line.find(delim)) <= std::string::npos and i < 3 and !line.empty())
		{
			token = line.substr(0, pos);
			std::cout << token << " : " << i << std::endl;
			line.erase(0, pos + delim.length());
			if (i == 0)
			{
				col = std::atoi(token.c_str());
			}
			else if (i == 1)
			{
				row = std::atoi(token.c_str());
			}
			else
			{
				val = std::atof(token.c_str());
			}
			i++;
		}
		if (i == 3)
			K(row - 1, col - 1) = val;
	}
	std::cout << K << std::endl;
}

LQR::~LQR()
{
}

void LQR::initVectors()
{
	xh = Eigen::Matrix<double, 1, 12>::Zero();
	xs = Eigen::Matrix<double, 1, 12>::Zero();
	u = Eigen::Matrix<double, 4, 1>::Zero();
}

void LQR::updatePos(double x, double y, double z)
{
	xh(0, 0) = x;
	xh(1, 0) = y;
	xh(2, 0) = z;
}
void LQR::updateVel(double x, double y, double z)
{
	xh(3, 0) = x;
	xh(4, 0) = y;
	xh(5, 0) = z;
}
void LQR::updateAndPos(double r, double p, double y)
{
	xh(6, 0) = r;
	xh(7, 0) = p;
	xh(8, 0) = y;
}
void LQR::updateAngVel(double r, double p, double y)
{
	xh(9, 0) = r;
	xh(10, 0) = p;
	xh(11, 0) = y;
}
void LQR::getCommand(double cmd[], double x, double y, double z,
										 double xd, double yd, double zd,
										 double r, double p, double yaw,
										 double rd, double pd, double yawd)
{
	xs << x, y, z, xd, yd, zd, r, p, yaw, rd, pd, yawd;
	u = K * (xs - xh);
	for (int i = 0; i < 4; i++)
		cmd[i] = K(i, 0);
}
} // namespace bsc_common