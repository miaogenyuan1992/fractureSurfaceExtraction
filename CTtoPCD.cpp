// pclitkdicomtopcd.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#pragma warning(disable:4996)

#include <iostream>
#include<algorithm>

#include<itkImage.h>
#include<itkPointSet.h>
#include<itkImageSeriesReader.h>
#include<itkGDCMImageIO.h>
#include<itkGDCMSeriesFileNames.h>
#include<itkDiscreteGaussianImageFilter.h>
#include<itkImageRegionIterator.h>
#include<itkNeighborhoodIterator.h>
#include<itkImageSliceIteratorWithIndex.h>
#include<itkImageSeriesWriter.h>


#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/common/common.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/features/principal_curvatures.h>
#include<pcl/features/normal_3d.h>
#include<pcl/filters/median_filter.h>
#include<pcl/surface/convex_hull.h>


using InternalpixelType = float;
constexpr unsigned int Dimention = 3;
using InternalimageType = itk::Image<InternalpixelType, Dimention>;
using Image2D = itk::Image<InternalpixelType, 2>;

using ReaderType = itk::ImageSeriesReader<InternalimageType>;
using ImageIOType = itk::GDCMImageIO;
using GaussianFilterType = itk::DiscreteGaussianImageFilter<InternalimageType, InternalimageType>;

using NameGeneratorType = itk::GDCMSeriesFileNames;
using neighborhoodIteratorType = itk::NeighborhoodIterator<InternalimageType>;
//using neighborhoodIteratorType = itk::NeighborhoodIterator<Image2D>;
using regionIteratorType = itk::ImageRegionIterator<InternalimageType>;
using slicerIteratorType = itk::ImageSliceIteratorWithIndex<InternalimageType>;



using writerType3D = itk::ImageSeriesWriter<InternalimageType, Image2D>;

//using PixelType = itk::Vector<InternalpixelType, 3>;
using PointsetType = itk::PointSet<InternalpixelType, Dimention>;
pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);

typedef struct PCURVATURE
{
	int index;
	float curvature;
}PCURVATURE;

struct POINT3F
{
	float x;
	float y;
	float z;
};

bool CompareByCurvature(const PCURVATURE& a, const PCURVATURE& b)
{
	return a.curvature < b.curvature;
}


std::vector<PCURVATURE> getModelCurvatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1)
{
	//法向量计算
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud1);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	ne.setSearchMethod(tree);
	//ne.setKSearch(10);
	ne.setRadiusSearch(8);
	ne.compute(*cloud_normal);

	//曲率计算
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curve(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setInputCloud(cloud1);
	pc.setInputNormals(cloud_normal);
	pc.setSearchMethod(tree);
	//pc.setKSearch(1000);
	pc.setRadiusSearch(8);
	pc.compute(*cloud_curve);

	std::vector<PCURVATURE> tempCV;
	POINT3F tempPoint;
	float curvature = 0;
	PCURVATURE pv;
	tempPoint.x = tempPoint.y = tempPoint.z = 0;
	for (int i = 0; i < cloud_curve->size(); i++)
	{
		//平均曲率
		//curvature = abs((*cloud_curve)[i].pc1 + (*cloud_curve)[i].pc2) / 2;
		curvature = (*cloud_curve)[i].pc1;
		//高斯曲率
		//curvature = (*cloud_curve)[i].pc1*(*cloud_curve)[i].pc2;
		pv.index = i;
		pv.curvature = curvature;
		tempCV.insert(tempCV.end(), pv);
	}
	return tempCV;
}

//凸包
pcl::PointCloud<pcl::PointXYZ>::Ptr surfaceHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr surfacehull(new pcl::PointCloud<pcl::PointXYZ>);
	hull.reconstruct(*surfacehull);
	return surfacehull;
}

int main()
{
	ReaderType::Pointer Reader = ReaderType::New();
	ImageIOType::Pointer DicomIO = ImageIOType::New();
	NameGeneratorType::Pointer NameGenerator = NameGeneratorType::New();
	Reader->SetImageIO(DicomIO);
	NameGenerator->SetUseSeriesDetails(true);
	NameGenerator->AddSeriesRestriction("0008|0021");
	NameGenerator->SetDirectory("F:\\PCL\\duicecanzhao\\femur");

	using SeriesIDContainer = std::vector<std::string>;
	const SeriesIDContainer & seriesUID = NameGenerator->GetSeriesUIDs();
	auto seriesItr = seriesUID.begin();
	auto seriesEnd = seriesUID.end();
	using FileNameContainer = std::vector<std::string>;
	FileNameContainer FileNames;
	std::string seriesIdentifier;
	while (seriesItr != seriesEnd)
	{
		seriesIdentifier = seriesItr->c_str();
		std::cout << seriesItr->c_str() << std::endl;
		FileNames = NameGenerator->GetFileNames(seriesIdentifier);
		++seriesItr;
	}

	Reader->SetFileNames(FileNames);

	try
	{
		Reader->Update();
	}
	catch (const itk::ExceptionObject & ex)
	{
		std::cout << ex << std::endl;

	}
	InternalimageType::SizeType imgSize = Reader->GetOutput()->GetLargestPossibleRegion().GetSize();
	std::cout << "reader done！Original size: " << imgSize << std::endl;

	InternalimageType::RegionType Region = Reader->GetOutput()->GetRequestedRegion();

	ReaderType::Pointer Reader1 = ReaderType::New();
	NameGenerator->SetDirectory("F:\\PCL\\duicecanzhao\\femur");
	const SeriesIDContainer & seriesUID1 = NameGenerator->GetSeriesUIDs();
	auto seriesItr1 = seriesUID1.begin();
	auto seriesEnd1 = seriesUID1.end();
	FileNameContainer FileNames1;
	std::string seriesIdentifier1;
	while (seriesItr1 != seriesEnd1)
	{
		seriesIdentifier1 = seriesItr1->c_str();
		std::cout << seriesItr1->c_str() << std::endl;
		FileNames1 = NameGenerator->GetFileNames(seriesIdentifier1);
		++seriesItr1;
	}

	Reader1->SetFileNames(FileNames1);

	try
	{
		Reader1->Update();
	}
	catch (const itk::ExceptionObject & ex)
	{
		std::cout << ex << std::endl;

	}
	InternalimageType::SizeType imgSize1 = Reader1->GetOutput()->GetLargestPossibleRegion().GetSize();
	std::cout << "reader1 done！Original size: " << imgSize1 << std::endl;

	GaussianFilterType::Pointer gaussianFilter = GaussianFilterType::New();
	gaussianFilter->SetInput(Reader1->GetOutput());
	gaussianFilter->SetSigma(1.0);
	gaussianFilter->Update();

	//空心化
	InternalimageType::Pointer image = Reader1->GetOutput();
	//InternalimageType::Pointer image = gaussianFilter->GetOutput();
	image->CopyInformation(Reader1->GetOutput());
	//image->SetRequestedRegion(Region);

	neighborhoodIteratorType::RadiusType radius;
	radius.Fill(1);
	neighborhoodIteratorType neighborIterator(radius, Reader->GetOutput(), Reader->GetOutput()->GetRequestedRegion());
	neighborhoodIteratorType::OffsetType offset1 = { -1,0,0 };
	neighborhoodIteratorType::OffsetType offset2 = { 1,0,0 };
	neighborhoodIteratorType::OffsetType offset3 = { 0,-1,0 };
	neighborhoodIteratorType::OffsetType offset4 = { 0,1,0 };

	regionIteratorType outputit(image, Region);

	for (neighborIterator.GoToBegin(), outputit.GoToBegin(); !neighborIterator.IsAtEnd(); ++neighborIterator, ++outputit)
	{
		float centerValue = neighborIterator.GetCenterPixel();
		if (centerValue != 0)
		{
			int valueScore = 0;
			/*for (int i = 0; i < neighborIterator.Size(); ++i)
			{
				InternalimageType::OffsetType offset = neighborIterator.GetOffset(i);
				float neighborValue = neighborIterator.GetPixel(offset);
				if (neighborValue == 0)
				{
					valueScore++;
				}
			}*/
			if (neighborIterator.GetPixel(offset1) == 0)
			{
				valueScore++;
			}
			if (neighborIterator.GetPixel(offset2) == 0)
			{
				valueScore++;
			}
			if (neighborIterator.GetPixel(offset3) == 0)
			{
				valueScore++;
			}
			if (neighborIterator.GetPixel(offset4) == 0)
			{
				valueScore++;
			}
			if (valueScore != 0)
			{
				outputit.Set(255);
			}
			else
			{
				outputit.Set(0);
			}
		}
	}

	////导出空心化后的模型
	//writerType3D::Pointer writer1 = writerType3D::New();
	//NameGenerator->SetOutputDirectory("F:\\PCL\\PointCloud\\fibula\\fibulaProximalBigHollow");
	//writer1->SetMetaDataDictionaryArray(Reader->GetMetaDataDictionaryArray());
	//writer1->SetImageIO(DicomIO);
	//writer1->SetFileNames(NameGenerator->GetOutputFileNames());
	//
	//writer1->SetInput(image);
	//writer1->Update();

	//采用ITK对DICOM模型进行表面平滑处理


	//CT转pcd
	//InternalimageType::Pointer image = Reader->GetOutput();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

	//InternalimageType::RegionType Region = image->GetRequestedRegion();
	regionIteratorType  it(image, Region);
	InternalimageType::IndexType Index;

	int i;

	for (it.GoToBegin(), i = 0; !it.IsAtEnd(); ++it, ++i)
	{
		if (it.Get() != 0)
		{
			InternalimageType::PointType phsicalPoint;
			image->TransformIndexToPhysicalPoint(it.GetIndex(), phsicalPoint);

			pcl::PointXYZ pclPoint;
			/*pclPoint.x = phsicalPoint[0];
			pclPoint.y = phsicalPoint[1];
			pclPoint.z = phsicalPoint[2];*/

			Index = it.GetIndex();
			//导出部分骨块
			/*if (Index[2] > 620)
				break;*/

			/*if (Index[2] < 283)
				continue;*/

			pclPoint.x = Index[0];
			pclPoint.y = Index[1];
			pclPoint.z = Index[2];

			cloud->points.push_back(pclPoint);
		}


	}


	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->resize(cloud->width*cloud->height);

	pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
	voxelGrid.setInputCloud(cloud);
	voxelGrid.setLeafSize(5.0f, 5.0f, 5.0f);
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
	voxelGrid.filter(*downsampledCloud);
	//downsampledCloud = cloud;

	/*cloud1->width = cloud1->points.size();
	cloud1->height = 1;
	cloud1->resize(cloud1->width*cloud1->height);*/
	std::cout << "pcdSaveDone.." << std::endl;

	pcl::io::savePCDFileASCII("F:/PCL/duicecanzhao/femur.pcd", *cloud);
	// pcl::io::savePCDFileASCII("F:/PCL/PointCloud/femurProximalDown.pcd", *downsampledCloud);

	std::cout << "pcdSaveDone1.." << std::endl;


	/*pcl::MedianFilter<pcl::PointXYZ> medianFIlter;
	medianFIlter.setInputCloud(cloud);
	medianFIlter.setWindowSize(5);
	pcl::PointCloud<pcl::PointXYZ>::Ptr medianCloud(new pcl::PointCloud<pcl::PointXYZ>);
	medianFIlter.filter(*medianCloud);*/

	
	std::vector<PCURVATURE> allcurvatures;
	allcurvatures = getModelCurvatures(downsampledCloud);
	//allcurvatures = getModelCurvatures(cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud1->width = downsampledCloud->width;
	cloud1->height = downsampledCloud->height;
	cloud1->is_dense = downsampledCloud->is_dense;
	cloud1->points.resize(downsampledCloud->points.size());
	//cloud1->resize(downsampledCloud->size());
	std::vector<PCURVATURE> allcurvaturesCopy = allcurvatures;


	int MaxCurvNumb = 1;
	int SecondCurvNumb = 2;
	int ThirdCurvNumb = 600;
	std::nth_element(allcurvaturesCopy.begin(), allcurvaturesCopy.begin() + allcurvaturesCopy.size() - MaxCurvNumb, allcurvaturesCopy.end(), CompareByCurvature);
	float curvatureMax = 0.0;
	float curvatureSec = 0.0;
	float curvatureThird = 0.0;
	curvatureMax = allcurvaturesCopy[allcurvaturesCopy.size() - MaxCurvNumb].curvature;
	curvatureSec = allcurvaturesCopy[allcurvaturesCopy.size() - SecondCurvNumb].curvature;
	curvatureThird = allcurvaturesCopy[allcurvaturesCopy.size() - ThirdCurvNumb].curvature;
	std::cout << MaxCurvNumb << " 个点处的曲率值为：" << curvatureMax << std::endl;
	std::cout << SecondCurvNumb << " 个点处的曲率值为：" << curvatureSec << std::endl;
	std::cout << ThirdCurvNumb << " 个点处的曲率值为：" << curvatureThird << std::endl;

	//找到最大曲率值和最小值
	float Maxcurvature = 0;
	float Mincurvature = 0;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud2->width = downsampledCloud->width;
	cloud2->height = downsampledCloud->height;
	cloud2->is_dense = downsampledCloud->is_dense;
	cloud2->points.resize(downsampledCloud->points.size());

	for (int i = 0; i < allcurvatures.size(); i++)
	{
		if (Maxcurvature < allcurvatures[i].curvature)
		{
			Maxcurvature = allcurvatures[i].curvature;

		}
		if (Mincurvature > allcurvatures[i].curvature)
		{
			Mincurvature = allcurvatures[i].curvature;

		}
	}

	std::cout << "Mincurvature:" << Mincurvature << "; Maxcurvature:" << Maxcurvature << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr dealCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//dealCloud = medianCloud;
	dealCloud = downsampledCloud;

	//根据曲率大小着色
	for (int i = 0; i < dealCloud->size(); ++i)
	{
		/*pcl::PointXYZRGB point;
		cloud1->points[i].x = downsampledCloud->points[i].x;
		cloud1->points[i].y = downsampledCloud->points[i].y;
		cloud1->points[i].z = downsampledCloud->points[i].z;*/

		uint8_t r = 255;
		uint8_t g = 255;
		uint8_t b = 255;
		pcl::PointXYZRGB point;

		//根据曲率大小将点云分为黑、红、绿、蓝
		/*if (allcurvatures[i].curvature < 0.15)
		{
			cloud1->points[i].r = 0;
			cloud1->points[i].g = 0;
			cloud1->points[i].b = 0;
		}

		if ((allcurvatures[i].curvature < 0.22) && (allcurvatures[i].curvature >= 0.15))
		{
			cloud1->points[i].r = 255;
			cloud1->points[i].g = 0;
			cloud1->points[i].b = 0;
		}
		if ((allcurvatures[i].curvature < 0.24)&& (allcurvatures[i].curvature >= 0.22))
		{
			cloud1->points[i].r = 0;
			cloud1->points[i].g = 255;
			cloud1->points[i].b = 0;
		}
		if (allcurvatures[i].curvature >= 0.24)
		{
			cloud1->points[i].r = 0;
			cloud1->points[i].g = 0;
			cloud1->points[i].b = 255;
		}*/

		//根据曲率大小将点云分为渐变色
		/*r = 255 * (allcurvatures[i].curvature + 1.0f) / 2.0f;
		g = 255 - r;
		b = 0;*/


		if (allcurvatures[i].curvature > 0.3)
		{
			cloud1->points[i].x = dealCloud->points[i].x;
			cloud1->points[i].y = dealCloud->points[i].y;
			cloud1->points[i].z = dealCloud->points[i].z;

			r = 255;
			g = 0;
			b = 0;

			cloud1->points[i].r = r;
			cloud1->points[i].g = g;
			cloud1->points[i].b = b;
			//std::cout << "点" << i << "的坐标：" << cloud1->points[i].x << "," << cloud1->points[i].y << "," << cloud1->points[i].z << std::endl;
		}
		else
		{
			cloud1->points[i].x = dealCloud->points[i].x;
			cloud1->points[i].y = dealCloud->points[i].y;
			cloud1->points[i].z = dealCloud->points[i].z;

			r = 0;
			g = 0;
			b = 255;

			cloud1->points[i].r = r;
			cloud1->points[i].g = g;
			cloud1->points[i].b = b;
		}

		if (allcurvatures[i].curvature == Maxcurvature)
		{
			cloud2->points[i].x = dealCloud->points[i].x;
			cloud2->points[i].y = dealCloud->points[i].y;
			cloud2->points[i].z = dealCloud->points[i].z;

			r = 0;
			g = 0;
			b = 0;

			cloud2->points[i].r = r;
			cloud2->points[i].g = g;
			cloud2->points[i].b = b;
		}

		/*cloud1->points[i].r = r;
		cloud1->points[i].g = g;
		cloud1->points[i].b = b;*/

		/*if (allcurvatures[i].curvature >= 0.25)
		{
			cloud1->points[i].r = 0;
			cloud1->points[i].g = 0;
			cloud1->points[i].b = 255;
		}
		else
		{
			cloud1->points[i].r = 220;
			cloud1->points[i].g = 220;
			cloud1->points[i].b = 240;
		}*/
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr surfaceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	surfaceCloud = surfaceHull(downsampledCloud);


	pcl::visualization::PCLVisualizer viewer("pcd Viewer");
	pcl::visualization::PCLVisualizer viewer1("test Viewer");
	pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("convexHull_calculation"));


	/*int v1 = 0;
	int v2 = 1;
	viewer.createViewPort(0, 0, 0.5, 1, v1);
	viewer.createViewPort(0.5, 0, 1, 1, v2);*/

	viewer.setBackgroundColor(0, 0, 0);
	viewer1.setBackgroundColor(1, 1, 1);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c_cloud(cloud, 0, 255, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c_cloud1(downsampledCloud, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud2);
	viewer.addPointCloud(downsampledCloud, c_cloud, "CT pcd");
	//viewer1.addPointCloud(downsampledCloud, c_cloud1, "Test pcd");
	viewer1.addPointCloud(cloud1, rgb, "Test pcd");
	viewer1.addPointCloud(cloud2, rgb2, "Test pcd2");
	//viewer.addPointCloud(cloud, c_cloud, "CT pcd");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "CT pcd");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Test pcd");
	viewer1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "Test pcd2");
	/*viewer.addCoordinateSystem(1.0);
	viewer.initCameraParameters();*/

	pcl::PointXYZ minpt, maxpt;
	pcl::getMinMax3D(*cloud, minpt, maxpt);

	Eigen::Vector3f center((maxpt.x + minpt.x) / 2, (maxpt.y + minpt.y) / 2, (maxpt.z + minpt.z) / 2);
	Eigen::Vector3f diff = maxpt.getVector3fMap() - minpt.getVector3fMap();
	float distance = diff.norm();

	viewer.setCameraPosition(center(0) + distance, center(1), center(2), center(0), center(1), center(2), 0.5, 0.1, 0.0);
	/*viewer.setCameraPosition(center(0), center(1), center(2) + distance, -0.7, -0.9, 0.2, v1);
	viewer.setCameraPosition(center(0), center(1), center(2) + distance, -0.7, -0.9, 0.2, v2);*/
	//viewer1.setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), -0.7, -0.9, 0.2);
	viewer1.setCameraPosition(center(0) + distance, center(1), center(2), center(0), center(1), center(2), 0.1, 0.1, 0.0);

	viewer2->setBackgroundColor(0, 0, 0);
	viewer2->addPointCloud<pcl::PointXYZ>(cloud, c_cloud, "convexHull_cloud");
	viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 1, "convexHull_cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		viewer1.spinOnce(100);
		viewer2->spinOnce(100);
	}

	return 0;
}

