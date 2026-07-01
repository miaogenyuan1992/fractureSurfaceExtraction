// complytest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//


#pragma warning(disable : 4996)
//#include <iostream>
#include<stack>
#include<vector>
#include<array>
#include<queue>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/common/pca.h>
#include<pcl/common/centroid.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<algorithm>
#include<pcl/features/normal_3d.h>
#include<pcl/features/principal_curvatures.h>

int iteratNum = 0;
int axisID = 0;
using arrayPoint = std::array<float, 2>;
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

struct PCATreeNode
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	Eigen::Vector3f axis;
	int depth;
	PCATreeNode* left;
	PCATreeNode* right;

	PCATreeNode(pcl::PointCloud<pcl::PointXYZ>::Ptr c, int d) :cloud(c), depth(d), left(nullptr), right(nullptr) {}
};


struct LocalPoint {
	float x, y;
	pcl::PointXYZ origin;
	bool operator<(const LocalPoint& p) const {
		return std::tie(x, y) < std::tie(p.x, p.y);
	}
};

std::map<LocalPoint, bool> uniquePointsMap; // 用于存储唯一坐标点


void buildLocalCoordinateSystem(
	const Eigen::Vector3f& axis_dir,
	const Eigen::Vector3f& centroid,
	Eigen::Vector3f& localX,
	Eigen::Vector3f& localY)
{

	Eigen::Vector3f ref_dir(0, 1, 0); // 参考方向
	if (axis_dir.cross(ref_dir).norm() < 1e-6) {
		ref_dir = Eigen::Vector3f(1, 0, 0); // 防止共线
	}

	localX = axis_dir.cross(ref_dir).normalized();
	localY = axis_dir.cross(localX).normalized();
}


void processSweptPlane(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const Eigen::Vector3f& start_centroid,
	const Eigen::Vector3f& end_centroid,
	pcl::PointCloud<pcl::PointXYZ>::Ptr swept_cloud,
	pcl::visualization::PCLVisualizer& viewer)
{
	const float step_size = 0.1f; // 平面移动步长
	const float plane_thickness = 0.05f; // 平面厚度
	const float grid_resolution = 0.01f; // 局部坐标系网格精度


	Eigen::Vector3f scan_dir = (end_centroid - start_centroid).normalized();


	kdtree.setInputCloud(cloud);


	for (float t = 0; t <= 1.0; t += step_size) {
		Eigen::Vector3f plane_center = start_centroid + t * (end_centroid - start_centroid);

	
		Eigen::Vector3f localX, localY;
		buildLocalCoordinateSystem(scan_dir, plane_center, localX, localY);


		pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
		plane->values.resize(4);
		plane->values[0] = scan_dir.x();
		plane->values[1] = scan_dir.y();
		plane->values[2] = scan_dir.z();
		plane->values[3] = -scan_dir.dot(plane_center);


		std::vector<int> indices;
		std::vector<float> distances;
		kdtree.radiusSearch(
			pcl::PointXYZ(plane_center.x(), plane_center.y(), plane_center.z()),
			2.0, 
			indices,
			distances
		);

	
		for (const auto& idx : indices) {
			const auto& p = cloud->points[idx];

			float dist = std::abs(scan_dir.x()*p.x + scan_dir.y()*p.y + scan_dir.z()*p.z + plane->values[3]);
			if (dist > plane_thickness) continue;


			Eigen::Vector3f vec(p.x - plane_center.x(),
				p.y - plane_center.y(),
				p.z - plane_center.z());
			float x_local = vec.dot(localX);
			float y_local = vec.dot(localY);


			LocalPoint grid_point{
				std::round(x_local / grid_resolution) * grid_resolution,
				std::round(y_local / grid_resolution) * grid_resolution,
				p
			};

			
			if (!uniquePointsMap.count(grid_point)) {
				uniquePointsMap[grid_point] = true;
				swept_cloud->push_back(p);
			}
		}


		std::string plane_id = "plane_" + std::to_string(t);
		viewer.addPlane(*plane, plane_center.x(), plane_center.y(), plane_center.z(), plane_id);
	}
}

void deleteTree(PCATreeNode* node)
{
	if (!node) return;
	deleteTree(node->left);
	deleteTree(node->right);
	delete node;
}


void recursiveSplit(PCATreeNode* node, int max_depth, pcl::visualization::PCLVisualizer& viewer)
{
	if (node->depth >= max_depth || node->cloud->size() < 3) return;
	iteratNum++;
	int axisColorR = 255;
	int axisColorG = 0;
	axisID++;

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
	std::string line_id = "axis" + std::to_string(axisID);
	//viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(start_point.x(), start_point.y(), start_point.z()), 
		//pcl::PointXYZ(end_point.x(), end_point.y(), end_point.z()), axisColorR, axisColorG, 0, line_id);

	std::vector<float> projections;
	for (const auto& p : *node->cloud)
	{
		Eigen::Vector3f pt = p.getVector3fMap() - centroid;
		projections.push_back(pt.dot(node->axis));
	}
	size_t mid = projections.size() / 2;
	std::nth_element(projections.begin(), projections.begin() + mid, projections.end());

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


	if (left_cloud->size() >= 3)
	{
		node->left = new PCATreeNode(left_cloud, node->depth + 1);
		recursiveSplit(node->left, max_depth, viewer);
	}
	if (right_cloud->size() >= 3)
	{
		node->right = new PCATreeNode(right_cloud, node->depth + 1);
		recursiveSplit(node->right, max_depth, viewer);
	}
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

pcl::PointCloud<pcl::PointXYZ>::Ptr generateCylinderCloud(float R, float r_out, float theta_start,
	float theta_end, float delta_theta, float delta_phi, float thickness)
{
	float delta_r = 0.2f;
	float r_in = r_out - thickness;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (float theta = theta_start; theta <= theta_end; theta += delta_theta)
	{
		Eigen::Vector3f center(R*cos(theta), R*sin(theta), 0);
		Eigen::Vector3f tangent(-sin(theta), cos(theta), 0);
		Eigen::Vector3f normal(cos(theta), sin(theta), 0);
		Eigen::Vector3f binormal = tangent.cross(normal);

		for (float r = r_in; r < r_out; r += delta_r)
		{
			for (float phi = 0; phi < 2 * M_PI; phi += delta_phi)
			{
				float x_local = r * cos(phi);
				float y_local = r * sin(phi);
				Eigen::Vector3f point_out = center + x_local * normal + y_local * binormal;
				cloud->push_back(pcl::PointXYZ(point_out.x(), point_out.y(), point_out.z()));

				/*float x_local_in = r_in * cos(phi);
				float y_local_in = r_in * sin(phi);
				Eigen::Vector3f point_in = center + x_local * normal + y_local * binormal;
				cloud->push_back(pcl::PointXYZ(point_in.x(), point_in.y(), point_in.z()));*/
			}
		}

	}

	cloud->width = cloud->size();
	cloud->height = 1;
	cloud->is_dense = true;

	return cloud;
}

bool contains(std::stack<arrayPoint>& stk, const arrayPoint& target)
{
	std::stack<arrayPoint> temp;
	bool found = false;
	while (!stk.empty())
	{
		arrayPoint top = stk.top();
		stk.pop();
		temp.push(top);

		if (abs(top[0] - target[0]) < 0.1)
		{
			if (abs(top[1] - target[1]) < 0.1)
			{
				found = true;
				break;
			}
		}
	}

	while (!temp.empty()) {
		stk.push(temp.top());
		temp.pop();
	}

	return found;
}

inline pcl::RGB HSVtoRGB(float h, float s, float v) {
	h = std::fmod(h, 360.0f);
	float c = v * s;
	float x = c * (1 - std::abs(std::fmod(h / 60.0f, 2) - 1));
	float m = v - c;

	float r, g, b;
	if (h < 60) { r = c; g = x; b = 0; }
	else if (h < 120) { r = x; g = c; b = 0; }
	else if (h < 180) { r = 0; g = c; b = x; }
	else if (h < 240) { r = 0; g = x; b = c; }
	else if (h < 300) { r = x; g = 0; b = c; }
	else { r = c; g = 0; b = x; }

	pcl::RGB color;
	color.r = (r + m) * 255;
	color.g = (g + m) * 255;
	color.b = (b + m) * 255;
	return color;
}

std::string formatFloat(float num, int precision = 2) {
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(precision) << num;
	return oss.str();
}

void createColorBand(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
	int point_count = 10, int width = 10) {
	for (int i = 0; i < point_count; ++i) {
		for (int j = 0; j < width; ++j)
		{
			pcl::PointXYZRGB point;
			point.x = i * 0.01f;  // X轴线性分布
			point.y = j * 0.01f;;          // Y轴固定

			// HSV色相渐变（红→绿→蓝）
			float hue = 240.0f * i / point_count;  // 0-240度渐变
			pcl::RGB color = HSVtoRGB(hue, 1.0f, 1.0f);

			point.r = color.r;
			point.g = color.g;
			point.b = color.b;
			cloud->push_back(point);
		}

	}

}


float pointToLineDistance(const pcl::PointXYZ& point,
	const pcl::PointXYZ& line_start,
	const pcl::PointXYZ& line_end) {
	Eigen::Vector3f v = Eigen::Vector3f(line_end.x, line_end.y, line_end.z) -
		Eigen::Vector3f(line_start.x, line_start.y, line_start.z);
	Eigen::Vector3f w = Eigen::Vector3f(point.x, point.y, point.z) -
		Eigen::Vector3f(line_start.x, line_start.y, line_start.z);

	float c1 = w.dot(v);
	if (c1 <= 0) return w.norm();

	float c2 = v.dot(v);
	if (c2 <= c1) return (Eigen::Vector3f(point.x, point.y, point.z) -
		Eigen::Vector3f(line_end.x, line_end.y, line_end.z)).norm();

	float b = c1 / c2;
	Eigen::Vector3f pb = Eigen::Vector3f(line_start.x, line_start.y, line_start.z) + b * v;
	return (Eigen::Vector3f(point.x, point.y, point.z) - pb).norm();
}


float calculateAngle(const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2) {
	float dot = vec1.dot(vec2);
	float norm1 = vec1.norm();
	float norm2 = vec2.norm();
	return std::acos(dot / (norm1 * norm2));
}


int main()
{
	float outer_radius = 3;
	float thick = 1;
	float inner_radius = outer_radius - thick;
	float start_angle = M_PI / 6;
	float end_angle = M_PI / 6;
	float height = start_angle - end_angle;

	auto cylinder = generateCylinderCloud(
		10,    // 转弯半径
		3,    // 圆柱半径
		0,    // 起始角度
		M_PI / 3,    // 终止角度
		0.02f,     // 轴线采样角度步长
		M_PI / 100,   // 圆柱采样角度步长
		1   // 圆柱厚度
	);

	pcl::PointCloud<pcl::PointXYZ>::Ptr bonecloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:/PCL/PointCloud/tibiaProximal.pcd", *bonecloud);

	pcl::visualization::PCLVisualizer viewer3("normalShow");
	pcl::visualization::PCLVisualizer viewer("axisShow");
	viewer.setBackgroundColor(0.05, 0.05, 0.05);
	viewer.addCoordinateSystem(1.0);
	//viewer.addPointCloud<pcl::PointXYZ>(cylinder);
	viewer.addPointCloud<pcl::PointXYZ>(bonecloud);

	viewer.addText(
		"Inner Radius:" + std::to_string(inner_radius) + "m\n" +
		"Outer Radius:" + std::to_string(outer_radius) + "m\n" +
		"Height:" + std::to_string(height) + "m", 10, 15, 12, 1.0, 1.0, 1.0, "stats"
	);

	int max_depth = 3;//***********************
	//PCATreeNode* root = new PCATreeNode(cylinder, 0);
	PCATreeNode* root = new PCATreeNode(bonecloud, 0);

	recursiveSplit(root, max_depth, viewer);

	//std::cout << iteratNum << std::endl;

	std::vector<Eigen::Vector3f> centroids;
	collectCentroid(root, centroids);

	int lineID = 0;
	int planeID = 0;
	std::vector<Eigen::Vector3f> tangent(centroids.size());
	pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pathPiont(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr swept_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cylinderCopy(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cylinder, *cylinderCopy);
	/*pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setMaxIterations(1000);*/

	Eigen::Vector3f vec;
	vec << 1, 1, 0;

	pcl::PointCloud<pcl::PointXYZ>::Ptr swept_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr swept_cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr swept_cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXY>::Ptr firstpiontCloud(new pcl::PointCloud<pcl::PointXY>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtedCloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointXY firstpoint2;
	firstpoint2.x = 0;
	firstpoint2.y = 0;
	firstpiontCloud->push_back(firstpoint2);

	pcl::PointXYZ pStart0(centroids[0].x() - 0.75*(centroids[1].x() - centroids[0].x()), centroids[0].y() - 0.75*(centroids[1].y() - centroids[0].y()), centroids[0].z() - 0.75*(centroids[1].z() - centroids[0].z()));
	pcl::PointXYZ pEnd0(centroids[0].x(), centroids[0].y(), centroids[0].z());

	viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pStart0, pEnd0, 255, 0, 0, "axisF");
	//viewer3.addLine<pcl::PointXYZ, pcl::PointXYZ>(pStart0, pEnd0, 255, 0, 0, "axisF");

	//法向量计算
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(bonecloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	ne.setSearchMethod(tree);
	ne.setKSearch(16);
	//ne.setRadiusSearch(8);
	ne.compute(*cloud_normal);

	//计算曲率
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curve(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setInputCloud(bonecloud);
	pc.setInputNormals(cloud_normal);
	pc.setSearchMethod(tree);
	pc.setKSearch(16);
	//pc.setRadiusSearch(8);
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
	//========================
	std::vector<float> cloudAng;
	int angcalNum = 0;
	for (auto& point : bonecloud->points)
	{
		float angles = 0;
		float min_distance = std::numeric_limits<float>::max();
		int nearest_line_idx = -1;

		// 找到最近的轴线
		for (size_t j = 0; j < centroids.size() - 1; j++) {
			pcl::PointXYZ pStart(centroids[j + 1].x(), centroids[j + 1].y(), centroids[j + 1].z());
			pcl::PointXYZ pEnd(centroids[j].x(), centroids[j].y(), centroids[j].z());
			float dist = pointToLineDistance(point, pStart, pEnd);
			if (dist < min_distance) {
				min_distance = dist;
				nearest_line_idx = j;
			}
		}

		if (nearest_line_idx != -1) {
			// 计算轴线方向向量
			Eigen::Vector3f axis_dir(
				centroids[nearest_line_idx].x() - centroids[nearest_line_idx + 1].x(),
				centroids[nearest_line_idx].y() - centroids[nearest_line_idx + 1].y(),
				centroids[nearest_line_idx].z() - centroids[nearest_line_idx + 1].z());

			// 获取当前点的法线向量
			Eigen::Vector3f normal_vector(
				cloud_normal->points[angcalNum].normal_x,
				cloud_normal->points[angcalNum].normal_y,
				cloud_normal->points[angcalNum].normal_z
			);

			// 计算法线与轴线的夹角
			angles = calculateAngle(normal_vector, axis_dir);
		}
		else {
			angles = 0.0f;
		}

		cloudAng.push_back(angles * 180.0 / M_PI);
		angcalNum++;
	}

	//==========================

	tangent[0] = centroids[1] - centroids[0];
	tangent[0].normalize();
	Eigen::Vector3f reference_dir(tangent[0].x(), tangent[0].y(), tangent[0].z());

	Eigen::Vector3f d0;
	for (float t = 0; t <= 1; t += 0.01)//-------------
	{
		d0.x() = pStart0.x + t * (pEnd0.x - pStart0.x);
		d0.y() = pStart0.y + t * (pEnd0.y - pStart0.y);
		d0.z() = pStart0.z + t * (pEnd0.z - pStart0.z);

		Eigen::Vector3f plane_center0(pStart0.x + t * (pEnd0.x - pStart0.x), pStart0.y + t * (pEnd0.y - pStart0.y), pStart0.z + t * (pEnd0.z - pStart0.z));


		int normalNum = 0;
		for (auto& point : bonecloud->points)
		{
			bool flag = false;

			// 获取当前点的法线
			Eigen::Vector3f normal_vector(
				cloud_normal->points[normalNum].normal_x,
				cloud_normal->points[normalNum].normal_y,
				cloud_normal->points[normalNum].normal_z
			);
			normalNum++;

			// 计算点积
			float dot_product = normal_vector.normalized().dot(reference_dir.normalized());

			// 计算夹角（弧度转角度）
			float angle_rad = std::acos(dot_product);
			float angle_deg = angle_rad * 180.0 / M_PI;
			//std::cout << angle_deg << std::endl;

			float dis = abs(point.x*tangent[0].x() + point.y*tangent[0].y() + point.z*tangent[0].z() - d0.dot(tangent[0])) / sqrt(tangent[0].x()*tangent[0].x() + tangent[0].y()*tangent[0].y() + tangent[0].z()*tangent[0].z());
			//float dis = abs(point.x*tangent[0].x() + point.y*tangent[0].y() + point.z*tangent[0].z() - d0.dot(tangent[0]));
			if (dis < 0.3)//**********************
			{
				swept_cloud->push_back(point);

				//构建局部坐标系
				Eigen::Vector3f localX0, localY0;
				buildLocalCoordinateSystem(tangent[0], plane_center0, localX0, localY0);

				// 转换到局部坐标系
				Eigen::Vector3f vec0(point.x - plane_center0.x(),
					point.y - plane_center0.y(),
					point.z - plane_center0.z());
				float x_local1 = vec0.dot(localX0);
				float y_local1 = vec0.dot(localY0);

				int loopnum0 = 0;
				//swept_cloud1->push_back(point);

				/*if ((x_local1 > 0) && (y_local1 > 0))
				{*/
				for (auto& point0 : firstpiontCloud->points)
				{
					loopnum0++;
					if (abs(point0.x - x_local1) < 1.2)//********************
					{
						if (abs(point0.y - y_local1) < 1.2)
						{
							//std::cout << "get!" << std::endl;
							flag = true;
							break;
						}

					}
					if (loopnum0 == firstpiontCloud->size())
					{
						//swept_cloud1->push_back(point);
						pcl::PointXY firstpoint0;
						firstpoint0.x = x_local1;
						firstpoint0.y = y_local1;
						firstpiontCloud->push_back(firstpoint0);
					}

				}
				if (flag)
				{
					continue;
				}

				//if (/*(tempCV[normalNum].curvature < 0.00)|| */(tempCV[normalNum].curvature < 0.09))
				//if ((abs(angle_deg) < 155 && abs(angle_deg) > 65)&&(tempCV[normalNum].curvature < 0.25))
				//{
				//	std::cout << tempCV[normalNum].curvature << ";"<< angle_deg << std::endl;
				//	//std::cout << angle_deg << std::endl;
				//	pcl::PointXYZRGB pointRGB;
				//	pointRGB.x = point.x;
				//	pointRGB.y = point.y;
				//	pointRGB.z = point.z;
				//	pointRGB.r = 255;
				//	pointRGB.g = 0;
				//	pointRGB.b = 0;
				//	swept_cloud2->push_back(pointRGB);
				//	continue;
				//}

				if ((angle_deg < 96 && angle_deg > 85)/*||(angle_deg < 180 && angle_deg > 100)*/)//96--85
				//if (tempCV[normalNum].curvature < 0.3/*&&tempCV[normalNum].curvature > 0.20*/)
				{
					/*std::cout << tempCV[normalNum].curvature << ";" << angle_deg << std::endl;
					std::cout << angle_deg << std::endl;*/
					pcl::PointXYZRGB pointRGB;
					pointRGB.x = point.x;
					pointRGB.y = point.y;
					pointRGB.z = point.z;
					pointRGB.r = 255;
					pointRGB.g = 0;
					pointRGB.b = 0;
					swept_cloud2->push_back(pointRGB);
					continue;
				}

				if (tempCV[normalNum].curvature < 0.3 && tempCV[normalNum].curvature > 0.25)
				{
					/*std::cout << tempCV[normalNum].curvature << ";" << angle_deg << std::endl;*/

					pcl::PointXYZRGB pointRGB;
					pointRGB.x = point.x;
					pointRGB.y = point.y;
					pointRGB.z = point.z;
					pointRGB.r = 0;
					pointRGB.g = 255;
					pointRGB.b = 0;
					swept_cloud3->push_back(pointRGB);
					continue;
				}

				//if (abs(angle_deg) < 92 && abs(angle_deg) > 82)
				//if (abs(angle_deg) < 92 && abs(angle_deg) > 83)
				//{
				//	std::cout << angle_deg << std::endl;
				//	pcl::PointXYZRGB pointRGB;
				//	pointRGB.x = point.x;
				//	pointRGB.y = point.y;
				//	pointRGB.z = point.z;
				//	pointRGB.r = 0;
				//	pointRGB.g = 255;
				//	pointRGB.b = 0;
				//	//std::cout << "get!" << std::endl;
				//	swept_cloud3->push_back(pointRGB);
				//	continue;
				//}

				////构建局部坐标系
				//Eigen::Vector3f localX0, localY0;
				//buildLocalCoordinateSystem(tangent[0], plane_center0, localX0, localY0);

				//// 转换到局部坐标系
				//Eigen::Vector3f vec0(point.x - plane_center0.x(),
				//	point.y - plane_center0.y(),
				//	point.z - plane_center0.z());
				//float x_local1 = vec0.dot(localX0);
				//float y_local1 = vec0.dot(localY0);
				//
				//int loopnum0 = 0;
				////swept_cloud1->push_back(point);

				///*if ((x_local1 > 0) && (y_local1 > 0))
				//{*/
				//for (auto& point0 : firstpiontCloud->points)
				//{
				//	loopnum0++;
				//	if (abs(point0.x - x_local1) < 0.01)//********************
				//	{
				//		if (abs(point0.y - y_local1) < 0.01)
				//		{
				//			//std::cout << "get!" << std::endl;
				//			continue;
				//		}

				//	}
				//	if (loopnum0 == firstpiontCloud->size())
				//	{
				//		swept_cloud1->push_back(point);
				//		pcl::PointXY firstpoint0;
				//		firstpoint0.x = x_local1;
				//		firstpoint0.y = y_local1;
				//		firstpiontCloud->push_back(firstpoint0);
				//	}
				//	
				//}
				/*}*/
				swept_cloud1->push_back(point);
			}
			/*else
			{
				std::cout << dis << std::endl;
			}*/
		}
	}

	for (size_t i = 0; i < centroids.size() - 7; ++i)
	{
		pcl::PointXYZ pStart(centroids[i].x(), centroids[i].y(), centroids[i].z());
		pcl::PointXYZ pEnd(centroids[i + 1].x(), centroids[i + 1].y(), centroids[i + 1].z());

		std::string line_id = "axis" + std::to_string(lineID);
		viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(pStart, pEnd, 255, 0, 0, line_id);
		//viewer3.addLine<pcl::PointXYZ, pcl::PointXYZ>(pStart, pEnd, 255, 0, 0, line_id);

		//======构建平面
		tangent[i] = centroids[i + 1] - centroids[i];
		tangent[i].normalize();
		plane_coeff->values.resize(4);
		plane_coeff->values[0] = tangent[i].x();
		plane_coeff->values[1] = tangent[i].y();
		plane_coeff->values[2] = tangent[i].z();
		plane_coeff->values[3] = -centroids[i].dot(tangent[i]);
		/*plane_coeff->values[0] = vec.x();
		plane_coeff->values[1] = vec.y();
		plane_coeff->values[2] = vec.z();
		plane_coeff->values[3] = -centroids[i].dot(vec);*/

		/*std::cout << i << ":" << tangent[i].x() << "," << tangent[i].y() << "," << tangent[i].z() << ";" << std::endl;
		std::cout << i << ":" << centroids[i].x() << "," << centroids[i].y() << "," << centroids[i].z() << ";" << std::endl;*/


		Eigen::Vector3f reference_dir(tangent[i].x(), tangent[i].y(), tangent[i].z());

		//=====直线段离散化，添加采样节点
		Eigen::Vector3f d;
		for (float t = 0; t <= 1; t += 0.01)//-------------
		{
			d.x() = pStart.x + t * (pEnd.x - pStart.x);
			d.y() = pStart.y + t * (pEnd.y - pStart.y);
			d.z() = pStart.z + t * (pEnd.z - pStart.z);

			Eigen::Vector3f plane_center = centroids[i] + t * (centroids[i + 1] - centroids[i]);

			int normalNum = 0;
			//for (auto& point : cylinder->points)
			for (auto& point : bonecloud->points)
			{
				bool flag = false;

				Eigen::Vector3f normal_vector;
				// 获取当前点的法线
				/*if (i < 1)
				{*/
				normal_vector.x() = cloud_normal->points[normalNum].normal_x;
				normal_vector.y() = cloud_normal->points[normalNum].normal_y;
				normal_vector.z() = cloud_normal->points[normalNum].normal_z;

				/*}
				else
				{
					normal_vector.x() = cloud_normal->points[i].normal_x;
					normal_vector.y() = cloud_normal->points[i].normal_y;
					normal_vector.z() = cloud_normal->points[i].normal_z;
				}*/

				normalNum++;

				// 计算点积
				float dot_product = normal_vector.normalized().dot(reference_dir.normalized());

				// 计算夹角（弧度转角度）
				float angle_rad = std::acos(dot_product);
				float angle_deg = angle_rad * 180.0 / M_PI;
				//std::cout << angle_deg << std::endl;
				//std::cout << tempCV[normalNum].curvature << std::endl;

				float dis = abs(point.x*tangent[i].x() + point.y*tangent[i].y() + point.z*tangent[i].z() - d.dot(tangent[i]));
				if (dis < 0.3)//**********************
				{
					swept_cloud->push_back(point);

					//构建局部坐标系
					Eigen::Vector3f localX1, localY1;
					buildLocalCoordinateSystem(tangent[i], plane_center, localX1, localY1);

					// 转换到局部坐标系
					Eigen::Vector3f vec(point.x - plane_center.x(),
						point.y - plane_center.y(),
						point.z - plane_center.z());
					float x_local1 = vec.dot(localX1);
					float y_local1 = vec.dot(localY1);

					arrayPoint firstPoint;
					firstPoint[0] = x_local1;
					firstPoint[1] = y_local1;


					int loopnum = 0;
					//swept_cloud1->push_back(point);

					/*if ((x_local1 > 0) && (y_local1 > 0))
					{*/
					for (auto& point1 : firstpiontCloud->points)
					{
						loopnum++;
						if (abs(point1.x - x_local1) < 1.2)//********************
						{
							if (abs(point1.y - y_local1) < 1.2)
							{
								flag = true;
								break;
							}

						}
						if (loopnum == firstpiontCloud->size())
						{
							//swept_cloud1->push_back(point);
							pcl::PointXY firstpoint1;
							firstpoint1.x = x_local1;
							firstpoint1.y = y_local1;
							firstpiontCloud->push_back(firstpoint1);
						}

					}

					if (flag)
					{
						continue;
					}

					//if ((angle_deg < 100 && angle_deg > 89)|| (angle_deg < 75 && angle_deg > 70))
					if ((angle_deg < 106 && angle_deg > 95)/* || (angle_deg < 85 && angle_deg > 50)*/)
					{
						pcl::PointXYZRGB pointRGB;
						pointRGB.x = point.x;
						pointRGB.y = point.y;
						pointRGB.z = point.z;
						pointRGB.r = 255;
						pointRGB.g = 0;
						pointRGB.b = 0;
						swept_cloud2->push_back(pointRGB);
						continue;
					}

					if ((tempCV[normalNum].curvature < 5 && tempCV[normalNum].curvature > 0.26) || (tempCV[normalNum].curvature < 0.18 && tempCV[normalNum].curvature > 0.0))
					{
						/*std::cout << tempCV[normalNum].curvature << ";" << angle_deg << std::endl;*/

						pcl::PointXYZRGB pointRGB;
						pointRGB.x = point.x;
						pointRGB.y = point.y;
						pointRGB.z = point.z;
						pointRGB.r = 0;
						pointRGB.g = 255;
						pointRGB.b = 0;
						swept_cloud3->push_back(pointRGB);
						continue;
					}


					swept_cloud1->push_back(point);
				}


			}

		}

		std::string plane_id = "plane" + std::to_string(planeID);
		viewer.addPlane(*plane_coeff, plane_id);

		lineID++;
		planeID++;
	}
	//pcl::io::savePCDFileASCII("F:/PCL/dealedPoint/tibiaProximalDown.pcd", *swept_cloud1);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr curvatureCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int curvNum = 0;
	for (auto& point : bonecloud->points)
	{
		pcl::PointXYZRGB pointRGB;
		pointRGB.x = point.x;
		pointRGB.y = point.y;
		pointRGB.z = point.z;
		if (tempCV[curvNum].curvature > 0 && tempCV[curvNum].curvature <= 0.03)
		{
			pointRGB.r = 255;
			pointRGB.g = 0;
			pointRGB.b = 0;
		}

		if (tempCV[curvNum].curvature > 0.03 && tempCV[curvNum].curvature <= 0.06)
		{
			pointRGB.r = 255;
			pointRGB.g = 97;
			pointRGB.b = 0;
		}

		if (tempCV[curvNum].curvature > 0.06 && tempCV[curvNum].curvature <= 0.12)
		{
			pointRGB.r = 255;
			pointRGB.g = 153;
			pointRGB.b = 18;
		}

		if (tempCV[curvNum].curvature > 0.12 && tempCV[curvNum].curvature <= 0.15)
		{
			pointRGB.r = 255;
			pointRGB.g = 255;
			pointRGB.b = 0;
		}

		if (tempCV[curvNum].curvature > 0.15 && tempCV[curvNum].curvature <= 0.18)
		{
			pointRGB.r = 127;
			pointRGB.g = 255;
			pointRGB.b = 0;
		}

		if (tempCV[curvNum].curvature > 0.18 && tempCV[curvNum].curvature <= 0.21)
		{
			pointRGB.r = 0;
			pointRGB.g = 255;
			pointRGB.b = 0;
		}

		if (tempCV[curvNum].curvature > 0.21 && tempCV[curvNum].curvature <= 0.24)
		{
			pointRGB.r = 0;
			pointRGB.g = 255;
			pointRGB.b = 217;
		}

		if (tempCV[curvNum].curvature > 0.24 && tempCV[curvNum].curvature <= 0.27)
		{
			pointRGB.r = 0;
			pointRGB.g = 255;
			pointRGB.b = 255;
		}

		if (tempCV[curvNum].curvature > 0.27 && tempCV[curvNum].curvature <= 0.3)
		{
			pointRGB.r = 65;
			pointRGB.g = 105;
			pointRGB.b = 225;
		}

		if (tempCV[curvNum].curvature > 0.3)
		{
			pointRGB.r = 0;
			pointRGB.g = 0;
			pointRGB.b = 255;
		}
		curvatureCloud->push_back(pointRGB);

		curvNum++;
	}

	//pcl::io::savePCDFileASCII("F:/PCL/PointCloud/curve/curvatureCloud.pcd", *curvatureCloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr angCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int angNum = 0;
	for (auto& point : bonecloud->points)
	{
		pcl::PointXYZRGB pointRGB;
		pointRGB.x = point.x;
		pointRGB.y = point.y;
		pointRGB.z = point.z;
		if (/*cloudAng[angNum] > 58 && */cloudAng[angNum] <= 72)
		{
			pointRGB.r = 255;
			pointRGB.g = 0;
			pointRGB.b = 0;
		}

		if (cloudAng[angNum] > 72 && cloudAng[angNum] <= 78)
		{
			pointRGB.r = 255;
			pointRGB.g = 97;
			pointRGB.b = 0;
		}

		if (cloudAng[angNum] > 78 && cloudAng[angNum] <= 84)
		{
			pointRGB.r = 255;
			pointRGB.g = 153;
			pointRGB.b = 18;
		}

		if (cloudAng[angNum] > 84 && cloudAng[angNum] <= 90)
		{
			pointRGB.r = 255;
			pointRGB.g = 255;
			pointRGB.b = 0;
		}

		if (cloudAng[angNum] > 90 && cloudAng[angNum] <= 96)
		{
			pointRGB.r = 127;
			pointRGB.g = 255;
			pointRGB.b = 0;
		}

		if (cloudAng[angNum] > 96 && cloudAng[angNum] <= 102)
		{
			pointRGB.r = 0;
			pointRGB.g = 255;
			pointRGB.b = 0;
		}

		if (cloudAng[angNum] > 102 && cloudAng[angNum] <= 108)
		{
			pointRGB.r = 0;
			pointRGB.g = 255;
			pointRGB.b = 217;
		}

		if (cloudAng[angNum] > 108 && cloudAng[angNum] <= 114)
		{
			pointRGB.r = 0;
			pointRGB.g = 255;
			pointRGB.b = 255;
		}

		if (cloudAng[angNum] > 114 && cloudAng[angNum] <= 120)
		{
			pointRGB.r = 65;
			pointRGB.g = 105;
			pointRGB.b = 225;
		}

		if (cloudAng[angNum] > 120)
		{
			pointRGB.r = 0;
			pointRGB.g = 0;
			pointRGB.b = 255;
		}
		angCloud->push_back(pointRGB);

		angNum++;
	}

	std::cout << firstpiontCloud->size() << std::endl;
	std::cout << swept_cloud1->size() << std::endl;

	pcl::visualization::PCLVisualizer viewer1("pointsShow");
	viewer1.setBackgroundColor(0.05, 0.05, 0.05);
	viewer1.addCoordinateSystem(1.0);
	//viewer1.addPointCloud<pcl::PointXYZ>(swept_cloud);
	viewer1.addPointCloud<pcl::PointXYZ>(swept_cloud1);

	pcl::visualization::PCLVisualizer viewer2("bipointsShow");
	viewer2.setBackgroundColor(0.05, 0.05, 0.05);
	viewer2.addCoordinateSystem(1.0);
	viewer2.addPointCloud<pcl::PointXYZ>(swept_cloud1, "cloud1");
	viewer2.addPointCloud<pcl::PointXYZRGB>(swept_cloud2, "cloud2");
	viewer2.addPointCloud<pcl::PointXYZRGB>(swept_cloud3, "cloud3");

	//pcl::visualization::PCLVisualizer viewer3("normalShow");
	viewer3.setBackgroundColor(1, 1, 1);
	viewer3.addCoordinateSystem(1.0);
	viewer3.addPointCloud<pcl::PointXYZ>(bonecloud, "bonecloud");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.69, 0.87, 0.9, "bonecloud");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "bonecloud");
	viewer3.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(bonecloud, cloud_normal, 2, 15, "normals");//第一个数字是每几个点生成一个箭头，第二个数字是箭头长度
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.25, 0.41, 1, "normals");
	viewer3.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, "normals");

	pcl::visualization::PCLVisualizer viewer4("pointsShow1");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudBand(new pcl::PointCloud<pcl::PointXYZRGB>);
	createColorBand(cloudBand, 200, 15);  // 生成200个点的色带

	int v1 = 0;
	int v2 = 1;
	/*viewer4.createViewPort(0.0, 0.2, 1.0, 1.0,v1);
	viewer4.createViewPort(0.0, 0.0, 1.0, 0.2, v2);*/
	viewer4.setBackgroundColor(1, 1, 1);
	//viewer4.addCoordinateSystem(1.0);
	viewer4.addPointCloud(curvatureCloud, "curvatureCloud", v1);
	//viewer4.addPointCloud(cloudBand, "CloudBand", v2);
	//viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "CloudBand", v2);
	//viewer4.setCameraPosition(0, 0, 0, 0, -1, 0, 0, 0, 1, v2);
	//viewer4.setCameraFieldOfView(0.5, v2);

	/*float showNum = 0;
	for (int i = 0; i < 10; i++)
	{
		viewer4.addText3D(formatFloat(showNum, 2),
			pcl::PointXYZ(0 + 0.2 * i, 0.2, 0),
			0.05, 0.0, 0.0, 0.0, "num_" + std::to_string(i), v2);
		showNum += 0.023;
	}*/

	pcl::visualization::PCLVisualizer viewer5("angShow");
	viewer5.addPointCloud(angCloud, "angCloud");
	viewer5.setBackgroundColor(1, 1, 1);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		viewer1.spinOnce();
		viewer2.spinOnce();
		viewer3.spinOnce();
		viewer4.spinOnce();
		viewer5.spinOnce();
	}

	deleteTree(root);

	return 0;
}


