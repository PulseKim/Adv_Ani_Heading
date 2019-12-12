#include "ThrowingBall.h"

Throw::Throw(SkeletonPtr &hubo, float time)
{
	mHubo = hubo;
	mMotionGraph =  std::make_unique<MotionGraph>(mHubo, time);
	mParallelVel = 3.0;
	mStartY = 0.5;
}

void Throw::throw_target_default(float target_height)
{
	float offset_ground = 0.105;
	float g = 9.81;
	float jump_height = mMotionGraph->jump_height() + mHubo->getBodyNode("head")->getCOM()[1];
	// std::cout << jump_height <<std::endl;
	if(jump_height > target_height){
		std::cout << "jump_height is " << jump_height <<std::endl;
		std::cerr << "Expected ball height higher than jump height" << std::endl;
	}
	float init_vel_y = sqrt(2 * g * (target_height- mStartY));
	mInitVel[0] = 0;
	mInitVel[1] = init_vel_y;
	mInitVel[2] = -mParallelVel;
	mTimeStep = sqrt(2 * (target_height  - mStartY) / g) + sqrt(2 * (target_height - jump_height) / g);
	mInitPos[0] = 0;
	mInitPos[1] = mStartY;
	mInitPos[2] = mParallelVel * mTimeStep;

}