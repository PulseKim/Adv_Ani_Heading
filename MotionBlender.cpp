#include "MotionBlender.h"

using namespace dart::math;

MotionBlender::MotionBlender(Eigen::VectorXd current_pose, Eigen::VectorXd next_pose)
{
	mCurrentPose = current_pose;
	mNextPose = next_pose;
	align_parameter();
}

void MotionBlender::align_parameter()
{
	Eigen::Vector3d root_rot = Eigen::Vector3d(mCurrentPose[0],mCurrentPose[1],mCurrentPose[2]);
	Eigen::Vector3d root_pos = Eigen::Vector3d(mCurrentPose[3],mCurrentPose[4],mCurrentPose[5]);
	Eigen::Quaterniond r_current = expToQuat(root_rot);
	Eigen::Quaterniond r_next = expToQuat(Eigen::Vector3d(mNextPose[0],mNextPose[1],mNextPose[2]));
	Eigen::Vector3d pos_temp_diff = root_pos -Eigen::Vector3d(mNextPose[3],mNextPose[4],mNextPose[5]);
	Eigen::Quaterniond u_q = Eigen::Quaterniond(0, pos_temp_diff[0], pos_temp_diff[1], pos_temp_diff[2]);
	u_q = r_next.inverse()* u_q * r_next;
	this-> pose_diff = u_q.vec();
	this-> rot_diff = r_next.inverse() * r_current;
}

Eigen::VectorXd MotionBlender::root_aligned_pose(Eigen::VectorXd input_pose)
{
	Eigen::Quaterniond ori_rot = expToQuat(Eigen::Vector3d(input_pose[0],input_pose[1],input_pose[2]));
	Eigen::Quaterniond u_quaternion;
	u_quaternion.w() = 0 ;
	u_quaternion.vec() = this->pose_diff;
	Eigen::Quaterniond alignment_pose =  ori_rot * u_quaternion * ori_rot.inverse();
	Eigen::Quaterniond alignment_rot = ori_rot * this->rot_diff;
	Eigen::Vector3d aligned_rot = quatToExp(alignment_rot);
	input_pose[0] = aligned_rot[0];
	input_pose[1] = aligned_rot[1];
	input_pose[2] = aligned_rot[2];
	input_pose[3] += alignment_pose.x();
	input_pose[5] += alignment_pose.z();
	return input_pose;
}
