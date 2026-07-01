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

// ===== 新增结构体和全局变量 =====
struct LocalPoint {
	float x, y;
	pcl::PointXYZ origin;
	bool operator<(const LocalPoint& p) const {
		return std::tie(x, y) < std::tie(p.x, p.y);
	}
};

std::map<LocalPoint, bool> uniquePointsMap; // 用于存储唯一坐标点

// ===== 局部坐标系构建函数 =====
void buildLocalCoordinateSystem(
	const Eigen::Vector3f& axis_dir,
	const Eigen::Vector3f& centroid,
	Eigen::Vector3f& localX,
	Eigen::Vector3f& localY)
{
	// 创建正交基
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
	const float step_size = 0.1f; 
	const float plane_thickness = 0.05f;

	Eigen::Vector3f scan_dir = (end_centroid - start_centroid).normalized();

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
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
			2.0, // 搜索半径
			indices,
			distances
		);

		// ------ 坐标转换与筛选 ------
		for (const auto& idx : indices) {
			const auto& p = cloud->points[idx];

			// 计算到平面距离
			float dist = std::abs(scan_dir.x()*p.x + scan_dir.y()*p.y + scan_dir.z()*p.z + plane->values[3]);
			if (dist > plane_thickness) continue;

			// 转换到局部坐标系
			Eigen::Vector3f vec(p.x - plane_center.x(),
				p.y - plane_center.y(),
				p.z - plane_center.z());
			float x_local = vec.dot(localX);
			float y_local = vec.dot(localY);

			// 离散化坐标
			LocalPoint grid_point{
				std::round(x_local / grid_resolution) * grid_resolution,
				std::round(y_local / grid_resolution) * grid_resolution,
				p
			};

			// 记录首次出现的点
			if (!uniquePointsMap.count(grid_point)) {
				uniquePointsMap[grid_point] = true;
				swept_cloud->push_back(p);
			}
		}

		// ------ 可视化当前平面 ------
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
