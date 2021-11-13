#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/conversions.h>
//#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/ModelCoefficients.h>
#include <windows.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>

// Types
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
//using namespace pcl::registration;
PointCloud<PointXYZ>::Ptr src, tgt;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;


int
main(int argc, char ** argv)
{
	// Parse the command line arguments for .pcd files
	std::vector<int> p_file_indices;
	p_file_indices = parse_file_extension_argument(argc, argv, ".pcd");
	if (p_file_indices.size() != 2)
	{
		print_error("Need one input source PCD file and one input target PCD file to continue.\n");
		print_error("Example: %s source.pcd target.pcd\n", argv[0]);
		return (-1);
	}

	// Load the files
	print_info("Loading %s as source and %s as target...\n", argv[p_file_indices[0]], argv[p_file_indices[1]]);
	src.reset(new PointCloud<PointXYZ>);
	tgt.reset(new PointCloud<PointXYZ>);
	if (loadPCDFile(argv[p_file_indices[0]], *src) == -1 || loadPCDFile(argv[p_file_indices[1]], *tgt) == -1)
	{
		print_error("Error reading the input files!\n");
		return (-1);
	}

	pcl::PointXYZ point;
	point = src->at(0);
	for (int i = 0; i < src->size(); i++)
	{
		if (src->at(i).y < point.y)
		{
			point = src->at(i);
		}
	}
	double final_area = std::numeric_limits<double>::min();
	double final_points = std::numeric_limits<double>::max();
	double increment = 0.1f;
	double height = round(point.y) + 1.8;
	double final_height = round(point.y) + 1.8;
	double fheight = round(point.y);
	pcl::PointXYZ point2;
	point2 = src->at(0);
	for (int i = 0; i < src->size(); i++)
	{
		if (src->at(i).y > point2.y) {
			point2 = src->at(i);
		}
	}


	while (height < point2.y)
	{
		PointCloudT::Ptr plan = boost::shared_ptr <PointCloudT>(new PointCloudT());

		for (int j = 0; j < src->size(); j++)
		{
			if (src->at(j).y > (height - 0.01) && src->at(j).y < (height + 0.01))
			{
				plan->push_back(src->at(j));
			}
		}
		cout << "plan" << plan->size() << endl;
		//for  convex hull
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PolygonMesh::Ptr mesh;
		pcl::ConvexHull<pcl::PointXYZ> chull;
		//chull.setDimension(3);
		chull.setInputCloud(plan);
		chull.setComputeAreaVolume(true);
		chull.reconstruct(*cloud_hull);
		double area = chull.getTotalArea();
		cout << "area" << area << endl;
		cout << "points" << plan->size() << endl;
		if (final_area < area)
		{
			if (final_points > plan->size())
			{
				final_height = height;
				final_area = area;
				final_points = plan->size();

			}
		}
		height = height + increment;
	}
	cout << "final height" << final_height << "difference" << final_height - round(point.y);
	PointCloudT::Ptr plan = boost::shared_ptr <PointCloudT>(new PointCloudT());
	

	for (int j = 0; j < src->size(); j++)
	{
		if (src->at(j).y > (final_height - 0.01) && src->at(j).y < (final_height + 0.01))
		{
			plan->push_back(src->at(j));
		}
	}


	pcl::PointXYZ point3;
	point3 = tgt->at(0);
	for (int i = 0; i < tgt->size(); i++)
	{
		if (tgt->at(i).y < point3.y)
		{
			point3 = tgt->at(i);
		}
	}

	float height2 = round(point3.y) + (final_height - round(point.y));
	//cout << "height2" << height2 << endl;
	PointCloudT::Ptr plan2 = boost::shared_ptr <PointCloudT>(new PointCloudT());
	for (int j = 0; j < tgt->size(); j++)
	{
		if (tgt->at(j).y > (height2 - 0.05) && tgt->at(j).y < (height2 + 0.05))
		{
			plan2->push_back(tgt->at(j));
		}
	}
	float final_x = 0.0;
	float final_z = 0.0;
	cout << "plan2" << plan2->size() << endl;
	float max_dist_b = 100000.0f;
	float final_mean = 1000000.0f;

	clock_t tic = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setLeafSize(1.0f, 1.0f, 1.0f);
	sor.setInputCloud(plan);
	sor.filter(*cloud_filtered);
	sor.setInputCloud(plan2);
	sor.filter(*cloud_filtered2);
	/*pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(cloud_filtered, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud_filtered2, 0, 0, 255);
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.addPointCloud(cloud_filtered2, cloud_color_handler, "cloud");
	viewer.addPointCloud(cloud_filtered, keypoints_color_handler, "keypoints");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}*/

	std::vector<float> scene_lines_theta;
	std::vector<float> Model_lines_theta;
	std::vector<vector<int>> inliers_scene;
	std::vector<vector<int>> inliers_model;
	int point_processed = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr rem_point(new pcl::PointCloud<pcl::PointXYZ>);
	for (int n = 0; n < cloud_filtered->size(); n++)
	{
		rem_point->push_back(cloud_filtered->at(n));
	}
	while (point_processed < 4)
	{
		pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr modelLine(
			new pcl::SampleConsensusModelLine<pcl::PointXYZ>(rem_point, true));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelLine);
		ransac.setDistanceThreshold(0.1);
		ransac.computeModel();
		std::vector<int> inliers1;
		ransac.getInliers(inliers1);
		inliers_scene.push_back(inliers1);
		pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
		for (int k = 0; k < inliers1.size(); k++)
		{
			inliers2->indices.push_back(inliers1[k]);
		}
		Eigen::VectorXf coefficents;
		ransac.getModelCoefficients(coefficents);
		pcl::PointCloud<pcl::PointXYZ>::Ptr show(new pcl::PointCloud<pcl::PointXYZ>);
		for (int k = 0; k < inliers2->indices.size(); k++)
		{
			show->push_back(rem_point->at(inliers2->indices[k]));
		}
		float sum_x = 0.0; float sum_z = 0.0; float sum_z_sq = 0.0; float sum_x_sq = 0.0; float sum_xz = 0.0; float A = 0.0; float B = 0.0;
		for (int l = 0; l < show->size(); l++)
		{
			sum_x += show->at(l).x;
			sum_z += show->at(l).z;
			sum_x_sq += pow((show->at(l).x), 2);
			sum_z_sq += pow((show->at(l).z), 2);
			sum_xz += ((show->at(l).x)*(show->at(l).z));
		}
		float A1 = pow((sum_x / (show->size())), 2);
		float A2 = pow((sum_z / (show->size())), 2);
		float A3 = sum_z_sq / (show->size());
		float A4 = sum_x_sq / (show->size());
		float B1 = sum_xz / (show->size());
		float B2 = (sum_x / (show->size()))*(sum_z / (show->size()));
		A = A1 - A2 + A3 - A4;
		B = B1 - B2;
		float theta;
		if (B < 0)
		{
			theta = atan((A / (2 * B)) + sqrt(pow(A / (2 * B), 2) + 1));
		}
		if (B > 0)
		{
			theta = atan((A / (2 * B)) - sqrt(pow(A / (2 * B), 2) + 1));
		}
		scene_lines_theta.push_back(theta);
		point_processed += 1;
		pcl::ExtractIndices<pcl::PointXYZ> eifilter(true);
		eifilter.setInputCloud(rem_point);
		eifilter.setNegative(true);
		eifilter.setIndices(inliers2);
		eifilter.filterDirectly(rem_point);
		/*pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(show, 255, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud_filtered, 0, 0, 255);
		viewer.setBackgroundColor(0.0, 0.0, 0.0);
		viewer.addPointCloud(cloud_filtered, cloud_color_handler, "cloud");
		viewer.addPointCloud(show, keypoints_color_handler, "keypoints");
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}*/

	}
	int point_processed2 = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr rem_point2(new pcl::PointCloud<pcl::PointXYZ>);
	for (int n = 0; n < cloud_filtered2->size(); n++)
	{
		rem_point2->push_back(cloud_filtered2->at(n));
	}
	while (point_processed2 < (cloud_filtered2->size())*0.9)
	{
		pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr modelLine(
			new pcl::SampleConsensusModelLine<pcl::PointXYZ>(rem_point2, true));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelLine);
		ransac.setDistanceThreshold(0.1);
		ransac.computeModel();
		std::vector<int> inliers1;
		ransac.getInliers(inliers1);
		inliers_model.push_back(inliers1);
		//cout << "inliers" << inliers1.size();
		pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
		for (int k = 0; k < inliers1.size(); k++)
		{
			inliers2->indices.push_back(inliers1[k]);
		}
		Eigen::VectorXf coefficents;
		ransac.getModelCoefficients(coefficents);
		pcl::PointCloud<pcl::PointXYZ>::Ptr show(new pcl::PointCloud<pcl::PointXYZ>);
		if (inliers2->indices.size() > 4)
		{
			for (int k = 0; k < inliers2->indices.size(); k++)
			{
				show->push_back(rem_point2->at(inliers2->indices[k]));
			}

			float sum_x = 0.0; float sum_z = 0.0; float sum_z_sq = 0.0; float sum_x_sq = 0.0; float sum_xz = 0.0; float A = 0.0; float B = 0.0;
			for (int l = 0; l < show->size(); l++)
			{
				sum_x += show->at(l).x;
				sum_z += show->at(l).z;
				sum_x_sq += pow((show->at(l).x), 2);
				sum_z_sq += pow((show->at(l).z), 2);
				sum_xz += ((show->at(l).x)*(show->at(l).z));
			}
			float A1 = pow((sum_x / (show->size())), 2);
			float A2 = pow((sum_z / (show->size())), 2);
			float A3 = sum_z_sq / (show->size());
			float A4 = sum_x_sq / (show->size());
			float B1 = sum_xz / (show->size());
			float B2 = (sum_x / (show->size()))*(sum_z / (show->size()));
			A = A1 - A2 + A3 - A4;
			B = B1 - B2;
			float theta;
			if (B < 0)
			{
				theta = atan((A / (2 * B)) + sqrt(pow(A / (2 * B), 2) + 1));
			}
			if (B > 0)
			{
				theta = atan((A / (2 * B)) - sqrt(pow(A / (2 * B), 2) + 1));
			}
			Model_lines_theta.push_back(theta);
		}
		point_processed2 += inliers2->indices.size();
		pcl::ExtractIndices<pcl::PointXYZ> eifilter(true);
		eifilter.setInputCloud(rem_point);
		eifilter.setNegative(true);
		eifilter.setIndices(inliers2);
		eifilter.filterDirectly(rem_point2);
		/*pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(show, 255, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud_filtered2, 0, 0, 255);
		viewer.setBackgroundColor(0.0, 0.0, 0.0);
		viewer.addPointCloud(cloud_filtered2, cloud_color_handler, "cloud");
		viewer.addPointCloud(show, keypoints_color_handler, "keypoints");
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}*/

	}
	float final_theta;
	for (int k = 0; k < Model_lines_theta.size(); k++)
	{
		float temp_theta = Model_lines_theta[k] - scene_lines_theta[0];
		cout << "temp_theta" << temp_theta;
		float temp_thata2 = temp_theta + M_PI;
		for (int t = 0; t < 2; t++)
		{
			if (t == 1)
			{
				temp_theta = temp_thata2;
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
			transform_2.rotate(Eigen::AngleAxisf(temp_theta, Eigen::Vector3f::UnitY()));
			pcl::transformPointCloud(*cloud_filtered2, *transformed_cloud, transform_2, false);
			/*pcl::visualization::PCLVisualizer viewer("PCL Viewer");
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(transformed_cloud, 255, 0, 0);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud_filtered, 0, 0, 255);
			viewer.setBackgroundColor(0.0, 0.0, 0.0);
			viewer.addPointCloud(cloud_filtered, cloud_color_handler, "cloud");
			viewer.addPointCloud(transformed_cloud, keypoints_color_handler, "keypoints");
			while (!viewer.wasStopped())
			{
				viewer.spinOnce();
			}*/

			for (int l = 0; l < inliers_model[k].size(); l++)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
				float x_trans = cloud_filtered->at(inliers_scene[0][1]).x - transformed_cloud->at(inliers_model[k][l]).x;
				float z_trans = cloud_filtered->at(inliers_scene[0][1]).z - transformed_cloud->at(inliers_model[k][l]).z;
				cout << "x_trans" << x_trans << "z_trans" << z_trans;
				Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
				transform_1.translation() << x_trans, 0, z_trans;
				pcl::transformPointCloud(*transformed_cloud, *transformed_cloud1, transform_1, false);
				pcl::KdTreeFLANN<pcl::PointXYZ> matchsearch;
				matchsearch.setInputCloud(transformed_cloud1);
				std::vector<float> distrances;
				for (size_t m = 0; m < cloud_filtered->size(); m++)
				{
					int k = 1;
					std::vector<int> neigh_indices(1);
					std::vector<float> neigh_sqr_dists(1);
					int found_neighs = matchsearch.nearestKSearch(cloud_filtered->at(m), k, neigh_indices, neigh_sqr_dists);
					if (found_neighs == 1)
					{
						distrances.push_back(neigh_sqr_dists[0]);
						cout << "distance" << neigh_sqr_dists[0];
					}
				}
				pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud3(new pcl::PointCloud<pcl::PointXYZ>());

				Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
				transform_3.rotate(Eigen::AngleAxisf(temp_theta, Eigen::Vector3f::UnitY()));
				transform_3.translation() << x_trans, 0, z_trans;
				pcl::transformPointCloud(*cloud_filtered2, *transformed_cloud3, transform_3, false);

				//cout << "final_x" << final_x << "finalz" << final_z << "fianl_theta" << final_theta << endl;
				//cout << "plan" << cloud_filtered << "plan2" << cloud_filtered2 << endl;
				/*pcl::visualization::PCLVisualizer viewer("PCL Viewer");
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(transformed_cloud3, 255, 0, 0);
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud_filtered, 0, 0, 255);
				viewer.setBackgroundColor(0.0, 0.0, 0.0);
				viewer.addPointCloud(cloud_filtered, cloud_color_handler, "cloud");
				viewer.addPointCloud(transformed_cloud3, keypoints_color_handler, "keypoints");
				while (!viewer.wasStopped())
				{
					viewer.spinOnce();
				}*/

				float mean = accumulate(distrances.begin(), distrances.end(), 0.0) / distrances.size();
				cout << "mean" << mean;
				if (final_mean > mean)
				{
					final_mean = mean;
					cout << "fianl_mean" << final_mean << endl;
					final_x = x_trans;
					final_z = z_trans;
					final_theta = temp_theta;
				}
			}
		}
	}
	//pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	//transformPointCloud(*cloud_filtered2, *transformed_cloud, transform_2, false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1(new pcl::PointCloud<pcl::PointXYZ>());

	Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
	transform_1.rotate(Eigen::AngleAxisf(final_theta, Eigen::Vector3f::UnitY()));
	transform_1.translation() << final_x, 0, final_z;
	transformPointCloud(*cloud_filtered2, *transformed_cloud1, transform_1, false);

	cout << "final_x" << final_x << "finalz" << final_z << "fianl_theta" << final_theta << endl;
	//cout << "plan" << cloud_filtered << "plan2" << cloud_filtered2 << endl;
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(transformed_cloud1, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud_filtered, 0, 0, 255);
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.addPointCloud(cloud_filtered, cloud_color_handler, "cloud");
	viewer.addPointCloud(transformed_cloud1, keypoints_color_handler, "keypoints");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

}