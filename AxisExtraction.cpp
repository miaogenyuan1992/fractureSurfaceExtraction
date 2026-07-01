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

	//创建子节点
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

//收集质心
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
