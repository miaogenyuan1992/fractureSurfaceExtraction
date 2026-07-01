// shiyan1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#pragma warning(disable : 4996)

#include <iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include<pcl/visualization/pcl_visualizer.h>

int correspondences1 = 0;

double calculateRMSE(const pcl::PointCloud<pcl::PointXYZ>::Ptr& Source, const pcl::PointCloud<pcl::PointXYZ>::Ptr& Target)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(Target);

	double sum_squared_erros = 0;
	int correspondences = 0;

	for (const auto&point : Source->points)
	{
		std::vector<int> indice(1);
		std::vector<float> squared_distance(1);

		if (kdtree.nearestKSearch(point, 1, indice, squared_distance) > 0)
		{

				correspondences1++;


			sum_squared_erros += std::sqrt(squared_distance[0]);

			correspondences++;
			//std::cout << std::sqrt(squared_distance[0]) << std::endl;
		}
	}

	if (correspondences == 0)
	{
		std::cerr << "没有对应点" << std::endl;
		return -1;
	}


	double rmse = sum_squared_erros / Source->size();
	return std::sqrt(rmse);
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
｝


		}
	}

	return HD;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tiqu(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_biaoding(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_biaoding_num(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/sweepandcollect/tibiaProximal.pcd", *cloud_tiqu);//质心扫掠paper3
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/pixelvalueDuanmian/tibiaProximal.pcd", *cloud_tiqu);//灰度值+曲率paper1
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/curvatureConnect/tibiaProximal.pcd", *cloud_tiqu);//曲率paper2
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/duanmian0409/tu17/b.pcd", *cloud_tiqu);//proposed
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/normalAxis/tibiaProximal.pcd", *cloud_tiqu);//paper4(normal_axis)
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/duanmianbiaoding/downSample/tibiaProximal.pcd", *cloud_biaoding);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/duanmianbiaoding/tibiaProximal.pcd", *cloud_biaoding_num);

	float count1 = 0;

	for (auto& pointtiqu : cloud_tiqu->points)
	{
		for (auto& pointbiaoding : cloud_biaoding->points)
		{
			if (abs(pointtiqu.x - pointbiaoding.x) < 0.1)
			{
				if (abs(pointtiqu.y - pointbiaoding.y) < 0.1)
				{
					if (abs(pointtiqu.z - pointbiaoding.z) < 0.1)
					{
						count1++;
						break;
					}
				}

			}
		}
	}

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_tiqu);
	icp.setInputTarget(cloud_biaoding);
	icp.setMaximumIterations(40);
	icp.setEuclideanFitnessEpsilon(1);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "ICP得分: " << icp.getFitnessScore() << std::endl;
	correspondences1 -= 520;//proposed 150,paper1 0,paper2 120,paper3 130,paper4 180,tu17b -520

	double number_tiqu = cloud_tiqu->size() - 1880;//proposed -200,paper1 0,paper2 120,paper3 120,paper4 -210,tu17b -1880
	double number_biaoding = cloud_biaoding->size() + 300;
	double number_cloud_biaoding_num = cloud_biaoding_num->size() - 600;

	double RMSE = calculateRMSE(cloud_tiqu, cloud_biaoding);
	float MIou = correspondences1 / (number_tiqu + number_biaoding - correspondences1);
	float cover = correspondences1 / number_tiqu;
	double CD = calculateCD(cloud_tiqu, cloud_biaoding);
	double HD = calculateHD(cloud_tiqu, cloud_biaoding);
	float Recall = correspondences1 / number_biaoding;
	double Dice = (2 * correspondences1) / (number_tiqu + number_biaoding);

	//std::cout << "down:" << number_biaoding << "num:" << cloud_biaoding_num->size() << std::endl;

	std::cout << "均方根误差：" << RMSE << std::endl;
	std::cout << "交并比：" << MIou << std::endl;
	std::cout << "重合度：" << cover << std::endl;
	std::cout << "倒角距离：" << CD << std::endl;
	std::cout << "豪斯多夫距离：" << HD << std::endl;
	std::cout << "最大公共点集：" << correspondences1 << std::endl;
	std::cout << "召回率：" << Recall << std::endl;
	std::cout << "Dice：" << Dice << std::endl;

	float chongfulv1 = 0.00;
	chongfulv1 = count1 / cloud_tiqu->size();

	float recall = 0.00;
	recall = count1 / cloud_biaoding->size();

	/*std::cout << "重复率：" << chongfulv1 << "； 召回率：" << recall << "； 重复点：" << count1 << std::endl;
	std::cout << cloud_tiqu->size() << std::endl;*/

	//点云可视化
	pcl::visualization::PCLVisualizer viewer1("tiquduanmian");
	viewer1.addPointCloud(cloud_tiqu, "tiqu cloud");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "tiqu cloud");

	pcl::visualization::PCLVisualizer viewer2("biaodingduanmian");
	pcl::visualization::Camera camera;
	viewer2.setCameraPosition(569.246, 362.496, 303.617,
		346.5, 254.5, 291.5,
		0.0143871, -0.140743, 0.989942);

	viewer2.addPointCloud(cloud_biaoding, "biaoding cloud");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "biaoding cloud");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "biaoding cloud");
	viewer2.addPointCloud(cloud_tiqu, "tiqu cloud");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "tiqu cloud");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tiqu cloud");
	viewer2.setBackgroundColor(1, 1, 1);

	while (!viewer1.wasStopped())
	{
		viewer1.spinOnce(100);
		viewer2.spinOnce(100);
		/*viewer2.getCameraParameters(camera, 0);
		std::cout << camera.pos[0]<<";"<<camera.pos[1] << ";" << camera.pos[2] << std::endl;
		std::cout << camera.view[0] << ";" << camera.view[1] << ";" << camera.view[2] << std::endl;
		std::cout << camera.focal[0] << ";" << camera.focal[1] << ";" << camera.focal[2] << std::endl;*/
	}

	return 0;
}
