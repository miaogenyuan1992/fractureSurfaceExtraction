// PCDShow.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#pragma warning(disable : 4996)

#include <iostream>
#include<pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include<pcl/visualization/pcl_visualizer.h>

void addPerpendicularPlanes(pcl::visualization::PCLVisualizer& viewer,
	const pcl::PointXYZ& start,
	const pcl::PointXYZ& end,
	std::string plane_id,
	double plane_size = 1) {

	Eigen::Vector3f axis_dir(end.x - start.x, end.y - start.y, end.z - start.z);
	axis_dir.normalize();

	float axis_length = std::sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2) + pow(end.z - start.z, 2));

	pcl::PointXYZ center;
	center.x = (start.x + end.x)/2;
	center.y = (start.y + end.y) / 2;
	center.z = (start.z + end.z) / 2;

	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	for (float u = -plane_size / 2; u <= plane_size / 2; u += plane_size / 10) {
		for (float v = -plane_size / 2; v <= plane_size / 2; v += plane_size / 10) {
		
			Eigen::Vector3f u_dir = axis_dir.unitOrthogonal();
			Eigen::Vector3f v_dir = axis_dir.cross(u_dir);

			pcl::PointXYZ pt;
			pt.x = center.x + u * u_dir.x() + v * v_dir.x();
			pt.y = center.y + u * u_dir.y() + v * v_dir.y();
			pt.z = center.z + u * u_dir.z() + v * v_dir.z();
			plane->push_back(pt);
		}
	}

		viewer.addPolygon<pcl::PointXYZ>(plane, 0.3, 0.3, 1.0, plane_id);
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, plane_id);
	
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_proximal(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_distal(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr curve_distal(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr curve_proximal(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr biaoding_distal(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr biaoding_proximal(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr distal(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr proximal(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/duanmian/femurProximalregistration.pcd", *cloud_proximal);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/duanmian/femurDistalregistration.pcd", *cloud_distal);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/curveduanmian/femurDistalCurve.pcd", *curve_distal);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/curveduanmian/femurProximalCurve.pcd", *curve_proximal);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/duanmianbiaoding/tibiaDistal.pcd", *biaoding_distal);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/duanmianbiaoding/tibiaProximal.pcd", *biaoding_proximal);
	//pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/tibiaDistal.pcd", *distal);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/tibiaProximal.pcd", *proximal);

	//点云可视化
	pcl::visualization::PCLVisualizer viewer1("duanmianProximalCompare");
	viewer1.addPointCloud(biaoding_proximal, "biaoding_proximal");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.53, 0.81, 0.92, "biaoding_proximal");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "biaoding_proximal");
	viewer1.addPointCloud(cloud_proximal, "proximal duanmiancloud");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.89, 0.79, 0.73, "proximal duanmiancloud");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "proximal duanmiancloud");
	viewer1.setBackgroundColor(255 / 255, 255 / 255, 255 / 255);

	pcl::visualization::PCLVisualizer viewer2("curveduanmianProximalCompare");
	viewer2.addPointCloud(biaoding_proximal, "biaoding_proximal");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.53, 0.81, 0.92, "biaoding_proximal");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "biaoding_proximal");
	viewer2.addPointCloud(curve_proximal, "curve_proximal");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.89, 0.09, 0.05, "curve_proximal");
	viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "curve_proximal");
	viewer2.setBackgroundColor(255 / 255, 255 / 255, 255 / 255);

	pcl::visualization::PCLVisualizer viewer3("duanmianDistalCompare");
	viewer3.addPointCloud(biaoding_distal, "biaoding_distal");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.53, 0.81, 0.92, "biaoding_distal");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "biaoding_distal");
	viewer3.addPointCloud(cloud_distal, "distal duanmiancloud");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.89, 0.09, 0.05, "distal duanmiancloud");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "distal duanmiancloud");
	viewer3.setBackgroundColor(255 / 255, 255 / 255, 255 / 255);

	pcl::visualization::PCLVisualizer viewer4("originduanmian");
	viewer4.addPointCloud(biaoding_distal, "biaoding_distal");
	viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "biaoding_distal");
	viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "biaoding_distal");
	viewer4.addPointCloud(biaoding_proximal, "biaoding_proximal");
	viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "biaoding_proximal");
	viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "biaoding_proximal");
	viewer4.setBackgroundColor(255 / 255, 255 / 255, 255 / 255);
	
	pcl::visualization::PCLVisualizer viewer5("fragmentspiontcloud");
	viewer5.addPointCloud(distal, "distal");
	viewer5.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "distal");
	viewer5.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.8, "distal");
	viewer5.addPointCloud(proximal, "proximal");
	viewer5.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.64, 0.75, 0.85, "proximal");
	viewer5.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.8, "proximal");
	viewer5.setBackgroundColor(255 / 255, 255 / 255, 255 / 255);

	


	while (!viewer1.wasStopped())
	{
		viewer1.spinOnce(100);
		viewer2.spinOnce(100);
		viewer3.spinOnce(100);
		viewer4.spinOnce(100);
		viewer5.spinOnce(100);
	}

	return 0;
}
