#ifndef _CONTROLLER_H_
#define	_CONTROLLER_H_

#include <iostream>
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include "dart/utils/utils.hpp"
using namespace dart::dynamics;


class Controller{
public:
	//Constructor
	Controller(const SkeletonPtr& finger);
	void jointControlSetter();
	void setTargetPosition(const Eigen::VectorXd& pose);
	void clearForces();
	void addSPDForces();
	void setFreeJointPosition();
	void setRootZero();
	void setAllZero();
	void addPDForces();
	Eigen::VectorXd getForces();


protected:

	Eigen::VectorXd mTargetPositions;
	//Finger model used to control
	SkeletonPtr mHubo;

	//Joint force to the fingers
	Eigen::VectorXd mForces;

	//control gains for proportional error terms in the PD controller
	Eigen::MatrixXd mKp;

	//Control gains for the derivative error terms in the PD controller
	Eigen::MatrixXd mKd;

};
#endif