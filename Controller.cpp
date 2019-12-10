#include "Controller.h"

const double rad = M_PI /180.0;

Controller::Controller(const SkeletonPtr& finger): mHubo(finger)
{
    this->jointControlSetter();
}

void Controller::jointControlSetter()
{
	int nDofs = mHubo->getNumDofs();
    int mag_Kp = 400;

    mForces = Eigen::VectorXd::Zero(nDofs);
    mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
    mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);

    for(std::size_t i = 0; i < nDofs; ++i){
        mKp(i,i) = mag_Kp;
        mKd(i,i) = 2 * std::sqrt(mag_Kp);
    }
    mag_Kp = 1500;
    for(std::size_t i = 3; i < 6; ++i){
        mKp(i,i) = mag_Kp;
        mKd(i,i) = 2 * std::sqrt(mag_Kp);
    }
    this->setTargetPosition(mHubo->getPositions());
}

void Controller::setTargetPosition(const Eigen::VectorXd& pose)
{
	mTargetPositions = pose;
}

void Controller::clearForces()
{
	mForces.setZero();
}

Eigen::VectorXd Controller::getForces()
{
    return mForces;
}

void Controller::addSPDForces()
{
	Eigen::VectorXd q = mHubo->getPositions();
    Eigen::VectorXd dq = mHubo->getVelocities();
    int cnt = 1;
    while(cnt !=0){
        cnt = 0;
        for(int i = 0; i < q.size(); i+= 3){
            if (i == 3)
                continue;
            else{
               Eigen::Vector3d current_axis = Eigen::Vector3d(q[i],q[i+1],q[i+2]);
               Eigen::Vector3d target_axis = Eigen::Vector3d(mTargetPositions[i],mTargetPositions[i+1],mTargetPositions[i+2] );
               float angle_dev = current_axis.norm() - target_axis.norm();
               if(angle_dev > M_PI){
                    current_axis -= 2*M_PI / 3 * Eigen::Vector3d(1,1,1);
                    // dq[i]-=2*M_PI;
                    cnt =1;
                }
                else if(angle_dev-mTargetPositions[i] < -M_PI){
                    current_axis += 2*M_PI / 3 * Eigen::Vector3d(1,1,1);
                    cnt =1;
                } 
            }
        }
    }

    Eigen::MatrixXd invM = (mHubo->getMassMatrix()
        + mKd * mHubo->getTimeStep()).inverse();
    Eigen::VectorXd p =
    -mKp * (q + dq * mHubo->getTimeStep() - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;
    Eigen::VectorXd qddot =
    invM * (-mHubo->getCoriolisAndGravityForces()
        + p + d + mHubo->getConstraintForces());
    
    mForces += p + d - mKd * qddot * mHubo->getTimeStep();
    mHubo->setForces(mForces);
    
}

void Controller::setFreeJointPosition()
{

    for(int i = 0; i < 6; i++){
        mHubo->setPosition(i, mTargetPositions(i));
        mHubo->setVelocity(i, 0);
    }
}

void Controller::addPDForces()
{
    Eigen::VectorXd q = mHubo->getPositions();
    Eigen::VectorXd dq = mHubo->getVelocities();
    int cnt = 1;
    while(cnt !=0){
        cnt = 0;
        for(int i = 6; i < q.size(); ++i){
            if(q[i]-mTargetPositions[i] > M_PI) {
                q[i]-=2*M_PI;
                dq[i]-=2*M_PI;
                cnt =1;
            }
            else if(q[i]-mTargetPositions[i] < -M_PI){
                q[i]+=2*M_PI;
                dq[i]+=2*M_PI;
                cnt =1;
            } 
        }
    }
    Eigen::VectorXd p = -mKp * (q - mTargetPositions);
    Eigen::VectorXd d = -mKd * dq;

    mForces += p+d;
    std::cout <<"q" << q.transpose() <<std::endl;
    std::cout <<"p" << p.transpose() <<std::endl;
    // std::cout << "d"  << d.transpose() << std::endl;
    mHubo->setForces(mForces);
}