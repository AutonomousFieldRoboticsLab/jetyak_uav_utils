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
 * This file implements a LQR controller
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

// [0_3,pdot_drone_dronebody,q_drone_world,qdot_drone_world]^T
void LQR::updateState(Eigen::Matrix<double, 12, 1> state)
{
	xh = state;
}

// [p_goal_dronebody,pdot_goal_dronebody,[0,0,yaw_goal_world],0_3]^T
Eigen::Matrix<double, 4, 1> LQR::getCommand(Eigen::Matrix<double, 12, 1> set)
{
	xs = set;
	u = K * (xh - xs);
	return u;
}
} // namespace bsc_common