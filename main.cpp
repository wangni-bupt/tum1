#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>

using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;



//由旋转平移矩阵计算旋转角度
void matrix2angle(Eigen::Matrix4f &result_trans, Eigen::Vector3f &result_angle)
{
	double ax, ay, az;
	if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1)
	{
		az = 0;
		double dlta;
		dlta = atan2(result_trans(0, 1), result_trans(0, 2));
		if (result_trans(2, 0) == -1)
		{
			ay = M_PI / 2;
			ax = az + dlta;
		}
		else
		{
			ay = -M_PI / 2;
			ax = -az + dlta;
		}
	}
	else
	{
		ay = -asin(result_trans(2, 0));
		ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
		az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
	}
	result_angle << ax, ay, az;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src_o(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("map_filter_target.pcd", *cloud_src_o);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt_o(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("map_filter.pcd", *cloud_tgt_o);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("map_sift_target.pcd", *cloud_src);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::io::loadPCDFile("map_fpfh_target.pcd", *fpfhs_src);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("map_sift.pcd", *cloud_tgt);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::io::loadPCDFile("map_fpfh.pcd", *fpfhs_tgt);
	
	//SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_src);
	scia.setInputTarget(cloud_tgt);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
	PointCloud::Ptr sac_result(new PointCloud);
	scia.align(*sac_result);
	std::cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();
	std::cout << sac_trans << endl;
	pcl::io::savePCDFileASCII("bunny_transformed_sac.pcd", *sac_result);

	//icp配准
	PointCloud::Ptr icp_result(new PointCloud);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt_o);
	//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(1);
	// 最大迭代次数
	icp.setMaximumIterations(50);
	// 两次变化矩阵之间的差值
	icp.setTransformationEpsilon(1e-10);
	// 均方误差
	icp.setEuclideanFitnessEpsilon(0.2);
	icp.align(*icp_result, sac_trans);



	std::cout << "ICP has converged:" << icp.hasConverged()
		<< " score: " << icp.getFitnessScore() << std::endl;
	Eigen::Matrix4f icp_trans;
	icp_trans = icp.getFinalTransformation();
	//cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
	std::cout << icp_trans << endl;
	//使用创建的变换对未过滤的输入点云进行变换
	pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);
	//保存转换的输入点云
	pcl::io::savePCDFileASCII("bunny_transformed_sac_ndt.pcd", *icp_result);

	//计算误差
	Eigen::Vector3f ANGLE_origin;
	ANGLE_origin << 0, 0, M_PI / 5;
	double error_x, error_y, error_z;
	Eigen::Vector3f ANGLE_result;
	matrix2angle(icp_trans, ANGLE_result);
	error_x = fabs(ANGLE_result(0)) - fabs(ANGLE_origin(0));
	error_y = fabs(ANGLE_result(1)) - fabs(ANGLE_origin(1));
	error_z = fabs(ANGLE_result(2)) - fabs(ANGLE_origin(2));
	cout << "original angle in x y z:\n" << ANGLE_origin << endl;
	cout << "error in aixs_x: " << error_x << "  error in aixs_y: " << error_y << "  error in aixs_z: " << error_z << endl;

	return (0);
}
