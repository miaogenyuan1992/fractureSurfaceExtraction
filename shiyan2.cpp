// registration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#pragma warning(disable : 4996)
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>

int correspondences1;

void alignLastTwoPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, Eigen::Matrix4f& transform1)
{
	// 获取两个点云的最后两个点
	pcl::PointXYZ p1_second_last = (*cloud1)[cloud1->size() - 2];
	pcl::PointXYZ p1_last = (*cloud1)[cloud1->size() - 1];
	pcl::PointXYZ p2_second_last = (*cloud2)[cloud2->size() - 2];
	pcl::PointXYZ p2_last = (*cloud2)[cloud2->size() - 1];

	// 计算两条线的向量
	Eigen::Vector3f line1 = p1_second_last.getVector3fMap() - p1_last.getVector3fMap();
	Eigen::Vector3f line2 = p2_last.getVector3fMap() - p2_second_last.getVector3fMap();

	// 计算旋转矩阵使line1与line2对齐
	line1.normalize();
	line2.normalize();
	Eigen::Vector3f axis = line1.cross(line2);
	float angle = acos(line1.dot(line2));
	Eigen::Matrix3f rotation = Eigen::AngleAxisf(angle, axis).toRotationMatrix();

	//Eigen::Vector3f rotateMove = rotation * line1;
	//double projectionCoeff = rotateMove.dot(line2) / line2.dot(line2);
	//Eigen::Vector3f targetpos = line2 * projectionCoeff;
	//// 计算平移向量使p1_last与p2_last重合
	////Eigen::Vector3f translation = line2 - rotation * line1;
	//Eigen::Vector3f translation = targetpos - rotateMove;
	Eigen::Vector3f translation = p2_last.getVector3fMap() - rotation * p1_last.getVector3fMap();

	// 构建变换矩阵
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform.block<3, 3>(0, 0) = rotation;
	transform.block<3, 1>(0, 3) = translation;

	transform1 = transform;
	// 应用变换到cloud1
	pcl::transformPointCloud(*cloud1, *cloud1, transform);

}


double calculateRMSE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& Source, const pcl::PointCloud<pcl::PointXYZ>::Ptr& Target, bool flag = true)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(Target);

	double sum_squared_erros = 0;
	int correspondences = 0;
	correspondences1 = 0;
	for (const auto&point : Source->points)
	{
		std::vector<int> indice(1);
		std::vector<float> squared_distance(1);

		if (kdtree.nearestKSearch(point, 1, indice, squared_distance) > 0)
		{
			if (flag)
			{
					correspondences1++;
			
			}
			else
			{
				correspondences1++;

			}

			sum_squared_erros += squared_distance[0];

			correspondences++;
			//std::cout << std::sqrt(squared_distance[0]) << std::endl;
		}
	}

	if (correspondences == 0)
	{
		std::cerr << "没有对应点" << std::endl;
		return -1;
	}


	double mse = sum_squared_erros / correspondences;
	return std::sqrt(mse);
}

double calculateCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& Source, const pcl::PointCloud<pcl::PointXYZ>::Ptr& Target)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSource;
	kdtreeSource.setInputCloud(Target);
	double dis1 = 0;
	int number1 = 0;
	number1 = Source->size();
	for (const auto&point : Source->points)
	{
		std::vector<float> square_distance1(1);
		std::vector<int> indice1(1);
		if (kdtreeSource.nearestKSearch(point, 1, indice1, square_distance1) > 0)
		{
			
				dis1 += square_distance1[0];
		

		}
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeTaregt;
	kdtreeTaregt.setInputCloud(Source);
	double dis2 = 0;
	int number2 = 0;
	number2 = Target->size();

	for (const auto&point : Target->points)
	{
		std::vector<int> indice2(1);
		std::vector<float> square_distance2(1);

		if (kdtreeTaregt.nearestKSearch(point, 1, indice2, square_distance2) > 0)
		{
				dis2 += square_distance2[0];
		

		}
	}

	return dis1 / number1 + dis2 / number2;
}

double calculateHD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& Source, const pcl::PointCloud<pcl::PointXYZ>::Ptr& Target)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(Target);

	double HD = 0;

	for (const auto&point : Source->points)
	{
		std::vector<int> indice(1);
		std::vector<float> square_distance(1);
		double dis = 0;
		if (kdtree.nearestKSearch(point, 1, indice, square_distance) > 0)
		{
				dis = std::sqrt(square_distance[0]);
				if (HD < dis)
				{
					HD = dis;
				}
		


		}
	}

	return HD;
}

int iteratNum = 0;
struct PCATreeNode
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	Eigen::Vector3f axis;
	int depth;
	PCATreeNode* left;
	PCATreeNode* right;

	PCATreeNode(pcl::PointCloud<pcl::PointXYZ>::Ptr c, int d) :cloud(c), depth(d), left(nullptr), right(nullptr) {}
};

//递归释放内存
void deleteTree(PCATreeNode* node)
{
	if (!node) return;
	deleteTree(node->left);
	deleteTree(node->right);
	delete node;
}

void collectCentroid(PCATreeNode* node, std::vector < Eigen::Vector3f >& centroids)
{
	if (!node) return;

	if (!node->left && !node->right)
	{
		pcl::PCA<pcl::PointXYZ> pca;
		pca.setInputCloud(node->cloud);
		Eigen::Vector3f centroid = pca.getMean().head<3>();
		centroids.push_back(centroid);
	}
	else
	{
		collectCentroid(node->left, centroids);
		collectCentroid(node->right, centroids);
	}
}

void recursiveSplit(PCATreeNode* node, int max_depth)
{
	if (node->depth >= max_depth || node->cloud->size() < 3) return;
	iteratNum++;
	int axisColorR = 255;
	int axisColorG = 0;


	//计算当前节点pca
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(node->cloud);
	Eigen::Vector3f centroid = pca.getMean().head<3>();
	Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
	Eigen::Vector3f eigenval = pca.getEigenValues();
	if (iteratNum < 2)
	{
		node->axis = eigenvectors.col(0);
	}
	else
	{
		node->axis = eigenvectors.col(2);
	}

	float length = 0.5 * sqrt(eigenval[0]);

	Eigen::Vector3f start_point = centroid.head<3>() - node->axis * length;
	Eigen::Vector3f end_point = centroid.head<3>() + node->axis  * length;

	if (iteratNum != 1)
	{
		axisColorR = 0;
		axisColorG = 255;
	}

	//计算投影中止
	std::vector<float> projections;
	for (const auto& p : *node->cloud)
	{
		Eigen::Vector3f pt = p.getVector3fMap() - centroid;//计算该点与中心点构成的向量
		projections.push_back(pt.dot(node->axis));//向量点乘轴线向量，使该点投影到轴线上。点云中所有投影点构成projection
	}
	size_t mid = projections.size() / 2;
	std::nth_element(projections.begin(), projections.begin() + mid, projections.end());
	//按大小排序确保projections.begin() + mid处的元素被排在数集中对应的位置
	float split_value = projections[mid];

	//分割点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (size_t i = 0; i < node->cloud->size(); ++i)
	{
		if (projections[i] < split_value)
		{
			left_cloud->push_back(node->cloud->points[i]);
		}
		else
		{
			right_cloud->push_back(node->cloud->points[i]);
		}
	}

	//创建子节点
	if (left_cloud->size() >= 3)
	{
		node->left = new PCATreeNode(left_cloud, node->depth + 1);
		recursiveSplit(node->left, max_depth);
	}
	if (right_cloud->size() >= 3)
	{
		node->right = new PCATreeNode(right_cloud, node->depth + 1);
		recursiveSplit(node->right, max_depth);
	}
}


int main()
{
	// 加载源点云和目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proximal(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_distal(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

	//=========proposed
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/duanmian0409/and/fibulaProximal/0.05and2.pcd", *cloud_target);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/duanmian0409/and/fibulaDistal/0.06and2.pcd", *cloud_source);//要移动和旋转的点云
	//================paper3
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/sweepandcollect/fibulaProximal.pcd", *cloud_target);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/sweepandcollect/fibulaDistal.pcd", *cloud_source);
	//===========paper1
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/pixelvalueDuanmian/fibulaProximal.pcd", *cloud_target);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/pixelvalueDuanmian/fibulaDistal.pcd", *cloud_source);
	//==========paper2
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/duanmian0409/curvature/fibulaProximal/0.04.pcd", *cloud_target);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/duanmian0409/curvature/fibulaDistal/0.06.pcd", *cloud_source);
	//=========paper4
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/normalAxis/fibulaProximal.pcd", *cloud_target);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/normalAxis/fibulaDistal.pcd", *cloud_source);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/fibulaProximal.pcd", *cloud_proximal);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/fibulaDistal.pcd", *cloud_distal);

	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/duicecanzhao/fibula.pcd", *cloud1);


	Eigen::Matrix4f transformCloud = Eigen::Matrix4f::Identity();
	//alignLastTwoPoints(cloud_source, cloud_target, transformCloud);//将两个骨块端面点云按照末端轴线先预对齐，轴线两端点在点云最后两位
																	 //实验端面标定点云与提取的端面点云配准时不需要此步骤

	//pcl::transformPointCloud(*cloud_distal, *cloud_distal, transformCloud);

	// 2. 创建变换矩阵
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

	//删除孤点
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter;
	rorfilter.setInputCloud(cloud_target);
	rorfilter.setRadiusSearch(7);
	rorfilter.setMinNeighborsInRadius(2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	rorfilter.filter(*target_filtered);

	rorfilter.setInputCloud(cloud_source);
	rorfilter.setRadiusSearch(7);
	rorfilter.setMinNeighborsInRadius(3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	rorfilter.filter(*source_filtered);

	// 创建ICP对象
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);
	/*icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);*/
	icp.setMaxCorrespondenceDistance(40);

	// 设置ICP参数
	//icp.setMaxCorrespondenceDistance(3);  // 最大对应距离
	icp.setMaximumIterations(80);            // 最大迭代次数
	//icp.setTransformationEpsilon(1e-8);      // 变换矩阵收敛阈值
	icp.setEuclideanFitnessEpsilon(0.1);       // 欧式距离收敛阈值

	// 执行配准
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_distalduanmian(new pcl::PointCloud<pcl::PointXYZ>);
	transformed_distalduanmian = Final.makeShared();

	// 输出结果
	if (icp.hasConverged())
	{
		std::cout << "ICP收敛" << std::endl;
		std::cout << "得分: " << icp.getFitnessScore() << std::endl;
		std::cout << "变换矩阵:\n" << icp.getFinalTransformation() << std::endl;
	}
	else
	{
		std::cout << "ICP未收敛" << std::endl;
		return -1;
	}



	double number_Proximal = cloud_target->size();
	double number_Distal = transformed_distalduanmian->size();

	double RMSE = calculateRMSE(transformed_distalduanmian, cloud_target);
	float MIou = correspondences1 / (number_Proximal + number_Distal - correspondences1);
	//float cover = correspondences1 / number_tiqu;
	double CD = calculateCD(cloud_target, transformed_distalduanmian);
	double HD = calculateHD(cloud_target, transformed_distalduanmian);

	std::cout << "均方根误差：" << RMSE << std::endl;
	std::cout << "交并比：" << MIou << std::endl;
	//std::cout << "重合度：" << cover << std::endl;
	std::cout << "倒角距离：" << CD << std::endl;
	std::cout << "豪斯多夫距离：" << HD << std::endl;
	std::cout << "最大公共点集：" << correspondences1 << std::endl;


	pcl::transformPointCloud(*cloud_distal, *cloud_distal, icp.getFinalTransformation());
	Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();


	pcl::PointCloud<pcl::PointXYZ>::Ptr mergedBoneCloud(new pcl::PointCloud<pcl::PointXYZ>);
	*mergedBoneCloud = *cloud_distal + *cloud_proximal;

	// 2. 计算质心并构建镜像平面
	Eigen::Vector4f centroid1, centroid2;
	pcl::compute3DCentroid(*cloud1, centroid1);
	pcl::compute3DCentroid(*mergedBoneCloud, centroid2);
	Eigen::Vector3f center = (centroid1.head<3>() + centroid2.head<3>()) / 2.0f;
	Eigen::Vector3f normal = (centroid2.head<3>() - centroid1.head<3>()).normalized();

	// 3. 镜像变换cloud1
	Eigen::Matrix4f mirror = Eigen::Matrix4f::Identity();
	mirror.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() - 2 * normal * normal.transpose();
	mirror.block<3, 1>(0, 3) = 2 * center.dot(normal) * normal;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mirrored(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud1, *mirrored, mirror);

	// 4. ICP配准
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icpShow;
	icpShow.setInputSource(mirrored);
	icpShow.setInputTarget(mergedBoneCloud);
	icpShow.setMaximumIterations(50);
	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
	icpShow.align(*aligned);
	//Eigen::Matrix4f transform = icpShow.getFinalTransformation();
	std::cout << "ICP Registration Score: " << icpShow.getFitnessScore() << std::endl;

	bool flag = false;

	double number_biaoding = mirrored->size();
	double number_merged = cloud_distal->size() + cloud_proximal->size();


	//pcl::io::savePCDFileASCII("F:/PCL/PointCloud/mergedbonecloud/0409/merged_cloud.pcd", *mergedBoneCloud);

	PCATreeNode* root1 = new PCATreeNode(aligned, 0);
	PCATreeNode* root2 = new PCATreeNode(mergedBoneCloud, 0);
	recursiveSplit(root1, 3);
	recursiveSplit(root2, 3);
	std::vector<Eigen::Vector3f> centroidPoints1;
	std::vector<Eigen::Vector3f> centroidPoints2;
	collectCentroid(root1, centroidPoints1);
	collectCentroid(root2, centroidPoints2);


	//接触面配准可视化
	pcl::visualization::PCLVisualizer viewer("ICP配准结果");
	viewer.addPointCloud(cloud_target, "Proximal cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "Proximal cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Proximal cloud");
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(cloud_target->points[cloud_target->size() - 1], cloud_target->points[cloud_target->size() - 2], 0.7, 0.09, 0.1, "proximalaxis");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, "proximalaxis");
	viewer.addPointCloud(cloud_source, "Distal cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "Distal cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Distal cloud");
	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(cloud_source->points[cloud_source->size() - 1], cloud_source->points[transformed_distalduanmian->size() - 2], 0.25, 0.41, 0.88, "distalaxis");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, "distalaxis");
	viewer.setBackgroundColor(1, 1, 1);
	//骨块配准可视化
	pcl::visualization::PCLVisualizer viewer1("骨块点云配准结果");
	viewer1.addPointCloud(cloud_proximal, "proximal cloud");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "proximal cloud");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "proximal cloud");
	viewer1.addPointCloud(cloud_distal, "distal cloud");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "distal cloud");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "distal cloud");

	viewer1.addPointCloud(aligned, "jiance cloud");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.53, 0.81, 0.92, "jiance cloud");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "jiance cloud");
	viewer1.setBackgroundColor(1, 1, 1);

	for (size_t i = 0; i < centroidPoints1.size() - 1; ++i)
	{
		pcl::PointXYZ pStart(centroidPoints1[i].x(), centroidPoints1[i].y(), centroidPoints1[i].z());
		pcl::PointXYZ pEnd(centroidPoints1[i + 1].x(), centroidPoints1[i + 1].y(), centroidPoints1[i + 1].z());
		std::string line_id = "axis1" + std::to_string(i);
		viewer1.addLine<pcl::PointXYZ, pcl::PointXYZ>(pStart, pEnd, 0.25, 0.41, 0.88, line_id);
		viewer1.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, line_id);

		pcl::PointXYZ pStart1(centroidPoints2[i].x(), centroidPoints2[i].y(), centroidPoints2[i].z());
		pcl::PointXYZ pEnd1(centroidPoints2[i + 1].x(), centroidPoints2[i + 1].y(), centroidPoints2[i + 1].z());
		std::string line_id1 = "axis2" + std::to_string(i);
		viewer1.addLine<pcl::PointXYZ, pcl::PointXYZ>(pStart1, pEnd1, 1, 0, 0, line_id1);
		viewer1.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, line_id1);
	}

	//合成近远端骨块
	pcl::visualization::PCLVisualizer viewer2("合成骨块点云");
	viewer2.addPointCloud(mergedBoneCloud, "merge cloud");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "merge cloud");

	//第一次配准轴对齐
	//合成近远端骨块
	pcl::visualization::PCLVisualizer viewer3("轴对齐结果");
	viewer3.addPointCloud(target_filtered, "Proximal cloud");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.1, 0.1, "Proximal cloud");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Proximal cloud");
	//viewer3.addLine<pcl::PointXYZ, pcl::PointXYZ>(cloud_target->points[cloud_target->size() - 1], cloud_target->points[cloud_target->size() - 2], 0.7, 0.09, 0.1, "proximalaxis");
	//viewer3.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, "proximalaxis");
	viewer3.addPointCloud(transformed_distalduanmian, "Distal cloud");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.8, 0.1, "Distal cloud");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Distal cloud");
	//viewer3.addLine<pcl::PointXYZ, pcl::PointXYZ>(cloud_source->points[cloud_source->size() - 1], cloud_source->points[transformed_distalduanmian->size() - 2], 0.25, 0.41, 0.88, "distalaxis");
	//viewer3.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3.0, "distalaxis");
	viewer3.setBackgroundColor(1, 1, 1);


	pcl::visualization::PCLVisualizer viewer4("ICP配准结果");
	viewer4.addPointCloud(target_filtered, "target cloud");
	//viewer4.addPointCloud(cloud_target, "target cloud");
	viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "target cloud");
	viewer4.setBackgroundColor(1, 1, 1);
	viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");


	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		viewer1.spinOnce(100);
		viewer2.spinOnce(100);
		viewer3.spinOnce(100);
		viewer4.spinOnce(100);
	}

	return 0;
}


