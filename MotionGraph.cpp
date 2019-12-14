#include "MotionGraph.h"
using namespace dart::math;

MotionGraph::MotionGraph(SkeletonPtr &biped, float time_step)
{
	mTimeStep = time_step;
	mHubo = biped;
	this -> walk();
	this -> jump();
	this -> run();

}

std::vector<Eigen::VectorXd> MotionGraph::walk_then_jump()
{
	auto blender = std::make_unique<MotionBlender>(mWalk[mWalk.size()-1], mJump[0]);
	std::vector<Eigen::VectorXd> aligned_jump;
	for(int i = 0 ; i < mJump.size(); ++i)
	{
		aligned_jump.push_back(blender->root_aligned_pose(mJump[i]));
	}
	std::vector<Eigen::VectorXd> interpolate;
	interpolate = this -> smoothTransition(mWalk[mWalk.size()-1], aligned_jump[0], 200);
	std::vector<Eigen::VectorXd> WalkJump;
	WalkJump.insert(WalkJump.end(), mWalk.begin(), mWalk.end());
	WalkJump.insert(WalkJump.end(), interpolate.begin(), interpolate.end());
	WalkJump.insert(WalkJump.end(), aligned_jump.begin(), aligned_jump.end());
	this -> jump_height();
	mJumpTime += mWalk.size();
	mJumpTime += interpolate.size();

	return WalkJump;
}

std::vector<Eigen::VectorXd> MotionGraph::run_then_jump()
{
	auto blender = std::make_unique<MotionBlender>(mRun[mRun.size()-1], mJump[0]);
	std::vector<Eigen::VectorXd> aligned_jump;
	for(int i = 0 ; i < mJump.size(); ++i)
	{
		aligned_jump.push_back(blender->root_aligned_pose(mJump[i]));
	}
	std::vector<Eigen::VectorXd> interpolate;
	interpolate = this -> smoothTransition(mRun[mRun.size()-1], aligned_jump[0], 200);
	std::vector<Eigen::VectorXd> RunJump;
	RunJump.insert(RunJump.end(), mRun.begin(), mRun.end());
	RunJump.insert(RunJump.end(), interpolate.begin(), interpolate.end());
	RunJump.insert(RunJump.end(), aligned_jump.begin(), aligned_jump.end());
	this -> jump_height();
	mJumpTime += mRun.size();
	mJumpTime += interpolate.size();

	return RunJump;
}


std::vector<Eigen::VectorXd> MotionGraph::smoothTransition(Eigen::VectorXd &current, Eigen::VectorXd &future, int blend_step)
{
	std::vector<Eigen::VectorXd> targets;
	for(int i = 1; i < blend_step ; ++i)
	{
		Eigen::VectorXd frame = Eigen::VectorXd::Zero(current.size());
		float t = float(i) / float(blend_step);
		for(int j = 0 ; j < current.size(); j+=3)
		{
			if(j == 3)
			{
				for(int k = 0; k < 3; ++k)
					frame[j+k] = (future[j+k] - current[j+k]) * t + current[j+k];
			}
			else
			{
				Eigen::Quaterniond quat_current = expToQuat(Eigen::Vector3d(current[j], current[j+1], current[j+2]));
				Eigen::Quaterniond quat_future = expToQuat(Eigen::Vector3d(future[j], future[j+1], future[j+2]));
				Eigen::Quaterniond quat_blend = quat_current.slerp(t, quat_future);
				Eigen::Vector3d blend_axis = quatToExp(quat_blend);
				for(int k = 0; k < 3; ++k)
					frame[j+k] = blend_axis[k];
			}
		}
		targets.push_back(frame);
	}
	return targets;
}

void MotionGraph::walk()
{
	std::string file_name = "16_31_walk.bvh";
	auto mbvh = std::make_unique<bvh>(file_name, mHubo, mTimeStep);
	mWalk = mbvh -> expMotionGetter();
 }

void MotionGraph::jump()
{
	std::string file_name = "16_01_jump.bvh";
	auto mbvh = std::make_unique<bvh>(file_name, mHubo, mTimeStep);
	mJump = mbvh -> expMotionGetter();
	for(int i = 0; i < 500; ++i)
		mJump.pop_back();
}

void MotionGraph::run()
{
	std::string file_name = "16_46_run&jog.bvh";
	auto mbvh = std::make_unique<bvh>(file_name, mHubo, mTimeStep);
	mRun = mbvh -> expMotionGetter();

}

float MotionGraph::walk_dist()
{
	Eigen::VectorXd start = mWalk[0];
	Eigen::VectorXd end = mWalk[mWalk.size()-1];
	float dist =  float(abs(start[5] - end[5])) / 5.0;
	return dist;
}

float MotionGraph::run_dist()
{
	Eigen::VectorXd start = mRun[0];
	Eigen::VectorXd end = mRun[mRun.size()-1];
	float dist =  float(abs(start[5] - end[5])) / 4.0;
	return dist;
}

float MotionGraph::total_walk_dist()
{
	Eigen::VectorXd start = mWalk[0];
	Eigen::VectorXd end = mWalk[mWalk.size()-1];
	float dist =  float(abs(start[5] - end[5]));
	return dist;
}

float MotionGraph::total_run_dist()
{
	Eigen::VectorXd start = mRun[0];
	Eigen::VectorXd end = mRun[mRun.size()-1];
	float dist =  float(abs(start[5] - end[5]));
	return dist;
}

float MotionGraph::jump_height()
{
	float start = mJump[0][4];
	float dist = 0;

	for(int i = 1; i < mJump.size(); ++i)
	{
		float curr = mJump[i][4] - start;
		if(curr > dist)
		{
			dist = curr;
			mJumpTime = i;
		}
	}
	// float head_pos = mHubo->getBodyNode("head")->getCOM()[1] - mHubo->getBodyNode("torso")->getCOM()[1] + 0.82 - 0.07;
	// std::cout << mHubo->getBodyNode("head")->getCOM()[1] << std::endl;	
	return dist;
}