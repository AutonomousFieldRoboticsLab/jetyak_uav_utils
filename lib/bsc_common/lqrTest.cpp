#include "include/lqr.h"
#include <eigen3/Eigen/Dense>
#include <iostream>


int main()
{
	const char *name = "lqrText.txt";
	bsc_common::LQR *lqr = new bsc_common::LQR(name);
	Eigen::Matrix<double,12,1> state=Eigen::Matrix<double,12,1>::Zero();
	Eigen::Matrix<double,12,1> set=Eigen::Matrix<double,12,1>::Zero();
	set(3)=1;
	lqr->updateState(state);
	std::cout << "\n\nCMD\n";
	std::cout << lqr->getCommand(set) << std::endl;
}
