#include <iostream>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/obj_io.h>

#define PI 3.14159265

void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [input files]\n\n"
            << "Input the path to DM and Camera Extrinsics\n"
            << "Example:\n"
            << "./dm2dpc dm.txt RT.txt\n"
            << "\n\n";
}

int main (int argc, char** argv)
{
	if (argc != 5)
	{
		printUsage (argv[0]);
		return 0;
	}
	else
	{
		std::fstream fs;
		std::string line;
		
		// Read the RT1 matrix from the file
		int j = 0;
		size_t k = 1;
		std::vector<std::string> strs;
		fs.open (argv[1], std::fstream::in);
		Eigen::Matrix4f RT1 = Eigen::Matrix4f::Identity();
		while (getline (fs,line))
    	{
    		boost::split(strs, line, boost::is_any_of(","));
			for (int i=0; i<strs.size(); i++)
			{
				RT1 (j,i) = atof(strs[i].c_str());
			}
			j++;
    	}
    	std::cout << RT1 << endl;
		fs.close();


		// Read the RT2 matrix from the file
		j = 0;
		fs.open (argv[3], std::fstream::in);
		Eigen::Matrix4f RT2 = Eigen::Matrix4f::Identity();
		while (getline (fs,line))
    	{
    		boost::split(strs, line, boost::is_any_of(","));
			for (int i=0; i<strs.size(); i++)
			{
				RT2 (j,i) = atof(strs[i].c_str());
			}
			j++;
    	}
    	std::cout << RT2 << endl;
		fs.close();

		// Read the DM from the files and convert to PC
		float depth;
		int x_res = 540;
		int y_res = 960;
		int fov_x = 70;
		int fov_y = 125;
		float focal_x = x_res/(2*tan(fov_x/2 * PI / 180.0));
		float focal_y = y_res/(2*tan(fov_y/2 * PI / 180.0));
		float u_center = x_res/2; 
		float v_center = y_res/2; 
		pcl::PointCloud<pcl::PointXYZ>::Ptr PC1 (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr PC2 (new pcl::PointCloud<pcl::PointXYZ> ());
		PC1->width = x_res;
		PC1->height = y_res;
		PC1->is_dense = true;
		PC1->points.resize(x_res*y_res);
		PC2->width = x_res;
		PC2->height = y_res;
		PC2->is_dense = true;
		PC2->points.resize(x_res*y_res);

		fs.open (argv[2], std::fstream::in);
		while (getline (fs,line))
		{
			boost::split(strs, line, boost::is_any_of(","));
			for (int i=0; i<strs.size(); i++)
			{
				depth = atof(strs[i].c_str());
				if (depth != 1)
				{
					PC1->points[k].x = depth * (i-u_center)/focal_x;
					PC1->points[k].y = depth * (j-v_center)/focal_y;
					PC1->points[k].z = depth;
					k++;
				}
			}
			j++;
		}
		pcl::transformPointCloud (*PC1, *PC1, RT1);
		fs.close();

		k = 0;
		j = 0;
		fs.open (argv[4], std::fstream::in);
		while (getline (fs,line))
		{
			boost::split(strs, line, boost::is_any_of(","));
			for (int i=0; i<strs.size(); i++)
			{
				depth = atof(strs[i].c_str());
				if (depth != 1)
				{
					PC2->points[k].x = depth * (i-u_center)/focal_x;
					PC2->points[k].y = depth * (j-v_center)/focal_y;
					PC2->points[k].z = depth;
					k++;
				}
			}
			j++;
		}
		pcl::transformPointCloud (*PC2, *PC2, RT2);
		fs.close();

		pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
		int v1(0);
		int v2(0);

		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (PC1, 255, 255, 255);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (PC2, 230, 20, 20); // Red
		viewer.addPointCloud (PC1, source_cloud_color_handler, "original_cloud", v1);		
		viewer.addPointCloud (PC2, transformed_cloud_color_handler, "transformed_cloud", v1);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, v1);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, v2);

		pcl::PointCloud<pcl::PointXYZ>::Ptr PC1_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr PC2_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud (PC1);
		sor.setLeafSize (0.01f, 0.01f, 0.01f);
		sor.filter (*PC1_filtered);

		sor.setInputCloud (PC2);
		sor.setLeafSize (0.01f, 0.01f, 0.01f);
		sor.filter (*PC2_filtered);

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(PC1_filtered);
		icp.setInputTarget(PC2_filtered);
		pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ> ());
		icp.align(*Final);
		std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Final_cloud_color_handler (Final, 255, 255, 255);
		viewer.addPointCloud(Final, Final_cloud_color_handler, "icp_cloud", v2);

		// Create the mesh from dpc
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (Final);
		n.setInputCloud (Final);
		n.setSearchMethod (tree);
		n.setKSearch (20);
		n.compute (*normals);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*Final, *normals, *cloud_with_normals);

		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		pcl::PolygonMesh triangles;

		gp3.setSearchRadius (0.025);
		gp3.setMu (2.5);
		gp3.setMaximumNearestNeighbors (100);
		gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
		gp3.setMinimumAngle(M_PI/18); // 10 degrees
		gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
		gp3.setNormalConsistency(false);

		gp3.setInputCloud (cloud_with_normals);
		gp3.setSearchMethod (tree2);
		gp3.reconstruct (triangles);

		std::vector<int> parts = gp3.getPartIDs();
		std::vector<int> states = gp3.getPointStates();

		pcl::io::saveOBJFile ("mesh.obj", triangles);

		while (!viewer.wasStopped ()) 
		{ 
			viewer.spinOnce ();
		}

		return 0;
	}

}