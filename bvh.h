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
	bvh(string &filename, SkeletonPtr &biped);
	vector<Eigen::VectorXd> motionGetter();
	float timeGetter();
	// ~bvh();

private:
	float time_step;
	vector<Eigen::VectorXd> mMotion;
};

#endif
