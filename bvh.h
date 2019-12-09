#ifndef _BVH_H_
#define _BVH_H_

#include "dart/dart.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>

using std::vector;
using std::string;
using std::ifstream;
using namespace dart::dynamics;


class bvh
{
public:
	bvh() = delete;
	bvh(string &filename, SkeletonPtr &biped, float sim_time);
	void linear_motion_gen();
	vector<Eigen::VectorXd> motionGetter();
	vector<Eigen::VectorXd> expMotionGetter();
	float timeGetter();
	// ~bvh();

private:
	float time_step;
	vector<Eigen::VectorXd> mMotion;
	vector<Eigen::VectorXd> mLinearExpandMotion;
	int interpolate_cnt;
};

#endif
