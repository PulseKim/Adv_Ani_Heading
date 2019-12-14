#ifndef _CLOTH_H_
#define _CLOTH_H_

#include <Eigen/Dense>
#include <vector>

class Cloth
{
public:
	Cloth() = delete;
	Cloth(	const Eigen::Vector3d &Pelvis
			const Eigen::Vector3d &Neck
			const Eigen::Vector3d &LShoulder
			const Eigen::Vector3d &LAnkle
			const Eigen::Vector3d &RShoulder
			const Eigen::Vector3d &RAnkle
			const Eigen::Vector3d &SkirtRoot);
	~Cloth();

	void SetPosition(	const Eigen::Vector3d &Pelvis
						const Eigen::Vector3d &Neck
						const Eigen::Vector3d &LShoulder
						const Eigen::Vector3d &LAnkle
						const Eigen::Vector3d &RShoulder
						const Eigen::Vector3d &RAnkle
						const Eigen::Vector3d &SkirtRoot);

	const std::vector<Eigen::Vector3d> &getVertices();

private:
	//TBD- (Jeonghun)

}

#endif
