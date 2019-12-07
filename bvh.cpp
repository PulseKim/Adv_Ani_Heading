#include "bvh.h"
#include <unistd.h>
#include <stdlib.h>
#define GetCurrentDir getcwd

using std::cout;
using std::cerr;
using std::endl;

using std::vector;
using std::string;
using std::ifstream;

using namespace dart::math;

string GetCurrentWorkingDir() 
{
  char buff[FILENAME_MAX];
  GetCurrentDir( buff, FILENAME_MAX );
  std::string current_working_dir(buff);
  return current_working_dir;
}

Eigen::Isometry3d getSE3ByTransV(Eigen::Vector3d trans)
{
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.translation() = trans;
    return iso;
}

bvh::bvh(string &filename, SkeletonPtr &biped)
{
	float scale = 0.01;
	string current = GetCurrentWorkingDir();
	string abs_file = current + "/../motion/" + filename;
	ifstream motionFile(abs_file);
	
	auto dofs = biped ->getDofs();
	vector<string> body_list;
	vector<vector<string>> body_channels;
	vector<Eigen::Isometry3d> trans_stack;
	trans_stack.push_back(Eigen::Isometry3d::Identity());
	int total_dofs = 0;

	for(int i =0; i < dofs.size();++i)
	{	
		cout << dofs[i]->getName() << endl;
		// cout << biped->getPositions()[i] << endl;
	}

	if(!motionFile){
		cout << "Expected valid file name" << endl;
	}
	string token;
	while(!motionFile.eof()){
        motionFile >> token;
        if(token == "OFFSET")
        {
        	float x;
        	motionFile >> x;
        	float y;
        	motionFile >> y;
        	float z;
        	motionFile >> z;
        	Eigen::Vector3d offset = Eigen::Vector3d(x, y, z);
        	offset *= scale;
            Eigen::Isometry3d trans = getSE3ByTransV(offset);
        	int size = trans_stack.size()-1;
        	trans_stack[size] = trans_stack[size] * trans;
        }
        else if(token == "ROOT" || token == "JOINT")
        {
        	string bodyName;
        	motionFile >> bodyName;
        	body_list.push_back(bodyName);
        }
        else if(token == "CHANNELS")
        {
        	int ndofs;
        	motionFile >> ndofs;
        	total_dofs += ndofs;
        	vector<string> single_channel;
        	for (int i =0 ; i < ndofs ; ++i)
        	{
        		string channel;
        		motionFile >> channel;
        		single_channel.push_back(channel);
        	}
        	body_channels.push_back(single_channel);
        }
        else if(token == "MOTION")
        {
        	motionFile >> token;
        	int num_frame;
        	motionFile >> num_frame;
        	motionFile >> token;
        	motionFile >> token;
            float dt;
        	motionFile >> dt;
            time_step = dt;
            cout << dt << endl;            
        	for(int i = 0 ; i < num_frame; ++i )
        	{
        		vector<float> one_frame;
        		for(int j = 0 ; j < total_dofs ; ++j)
        		{
        			float angle;
        			motionFile >> angle;
        			one_frame.push_back(angle);
        		}
                int cnt = 0;
        		Eigen::VectorXd pose = Eigen::VectorXd::Zero(biped->getPositions().size());
                for (int j =0; j < body_list.size() ; ++j)
                {
                    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
                    bool isTranslate = false;
                    auto current_channel  = body_channels[j];
                    for(int k = 0; k < current_channel.size(); k++)
                    {
                        float value = one_frame[cnt];
                        if (current_channel[k] == "Xposition")
                        {
                            isTranslate = true;
                            value = scale * value;
                            tf = tf * getSE3ByTransV(Eigen::Vector3d(value, 0 ,0));
                        }
                        else if (current_channel[k] == "Yposition")
                        {
                            isTranslate = true;
                            value = scale * value;
                            tf = tf * getSE3ByTransV(Eigen::Vector3d(0, value ,0));
                        }
                        else if (current_channel[k] == "Zposition")
                        {
                            isTranslate = true;
                            value = scale * value;
                            tf = tf * getSE3ByTransV(Eigen::Vector3d(0, 0, value));
                        }
                        else if (current_channel[k] == "Xrotation")
                        {
                            value = value * M_PI / 180.0;
                            // Eigen::Isometry3d rot = expAngular(Eigen::Vector3d(value, 0 ,0));
                            tf = tf * expAngular(Eigen::Vector3d(value, 0 ,0));
                        }
                        else if (current_channel[k] == "Yrotation")
                        {
                            value = value * M_PI / 180.0;
                            tf = tf * expAngular(Eigen::Vector3d(0, value ,0));
                        }
                        else if (current_channel[k] == "Zrotation")
                        {
                            value = value * M_PI / 180.0;
                            tf = tf * expAngular(Eigen::Vector3d(0 ,0, value));
                        }
                        cnt++;
                    }
                    Eigen::Vector3d w = logMap(tf.linear());
                    if (isTranslate)
                    {
                        string x_rot  = body_list[j] + "_rot_x";
                        int index = -1;
                        for(int k =0; k < dofs.size();++k)
                        {
                            if(dofs[k]->getName() == x_rot)
                            {
                                index = k;
                                break;
                            }
                        }
                        pose[index+3] = tf.translation()[0];
                        pose[index+4] = tf.translation()[1];
                        pose[index+5] = tf.translation()[2];
                        pose[index] = w[0];
                        pose[index+1] = w[1];
                        pose[index+2] = w[2];
                    }
                    else
                    {
                        string x_rot  = body_list[j] + "_x";
                        int index = -1;
                        for(int k =0; k < dofs.size();++k)
                        {
                            if(dofs[k]->getName() == x_rot)
                            {
                                index = k;
                                break;
                            }
                        }
                        if(index != -1)
                        {
                            pose[index] = w[0];
                            pose[index+1] = w[1];
                            pose[index+2] = w[2];
                        }
                    }
                }
                mMotion.push_back(pose);
        	}
        }
    }
    motionFile.close();
}

float bvh::timeGetter()
{
    return time_step;
}
vector<Eigen::VectorXd> bvh::motionGetter()
{
    return mMotion;
}