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
