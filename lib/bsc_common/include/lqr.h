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
 * This class provides an interface for a LQR controller
 * 
 * Author: Brennan Cain
 */
#ifndef BSC_COMMON_LQR_
#define BSC_COMMON_LQR_
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <cstdlib>

namespace bsc_common
{
class LQR
{
private:
	/* 
	K*(xh-xs)=u
	K(4,12) is the LQR Matrix
	xh(1,12) is the estimate of the state
	xs(1,12) is the setpoint of the state
	u(1,4) is the command output

	*/
	Eigen::Matrix<double, 4, 12> K;
	Eigen::Matrix<double, 12, 1> xh;
	Eigen::Matrix<double, 12, 1> xs;
	Eigen::Matrix<double, 4, 1> u;

	/** initVectors
	 * initialize the vectors as zeroes.
	 */
	void initVectors();

public:
	/** Constructor
	 * Takes a 4x12 Matrix and saves it as the LQR K matrix
	 */
	LQR(Eigen::Matrix<double, 4, 12> K);

	/** Constructor
	 * Parses a file to build the K matrix
	 * format:
	 * column row value
	 *    .    .    .
	 *    .    .    .
	 */
	LQR(std::string file);
	~LQR();

	// [0_3,pdot_drone_dronebody,q_drone_world,qdot_drone_world]^T
	void updateState(Eigen::Matrix<double, 12, 1> state);

	// [p_goal_dronebody,pdot_goal_dronebody,[0,0,yaw_goal_world],0_3]^T
	Eigen::Matrix<double, 4, 1> getCommand(Eigen::Matrix<double, 12, 1> set);
};
} // namespace bsc_common
#endif