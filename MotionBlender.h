#ifndef _MOTIONBLENDER_H_
#define _MOTIONBLENDER_H_

#include "dart/dart.hpp"

using namespace dart::dynamics;


class MotionBlender
{
public:
	MotionBlender(Eigen::VectorXd current_pose, Eigen::VectorXd next_pose);
	void align_parameter();
	Eigen::VectorXd root_aligned_pose(Eigen::VectorXd input_pose);
	

private:
	Eigen::VectorXd mCurrentPose;
	Eigen::VectorXd mNextPose;
	Eigen::Vector3d pose_diff;
	Eigen::Quaterniond rot_diff;
};

#endif
