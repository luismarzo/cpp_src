#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/luis/cpp_src/Interpolation/src/maps/strimona_bridge_spam2.pcd", *rgbcloud) == -1)
  {
    PCL_ERROR ("Couldn't read file .pcd \n");
    return (-1);
  }
  
    uint8_t r1(15), g1(15), b1(255);
      pcl::PointXYZRGB point_1,point_2;
      point_1.x = 0;
      point_1.y = 0;
      point_1.z = 2;
      
      point_2.x = 0;
      point_2.y = 0;
      point_2.z = 3;
  
        uint32_t rgb_1 = (static_cast<uint32_t>(r1) << 16 |
              static_cast<uint32_t>(g1) << 8 | static_cast<uint32_t>(b1));
      point_1.rgb = *reinterpret_cast<float*>(&rgb_1);
      point_2.rgb = *reinterpret_cast<float*>(&rgb_1);
      pcl::PointXYZ pt1 (1,0,2);
      pcl::PointXYZ pt2 (1,0,3);

      rgbcloud->points.push_back (point_1);
      rgbcloud->points.push_back (point_2);
  
//---------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/luis/cpp_src/Interpolation/src/maps/Bridge_Algodonales.pcd", *cloud) == -1) //* load the file 
  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/luis/cpp_src/Interpolation/src/maps/strimona_bridge_spam2.pcd", *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file .pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  /*for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;*/

//-----------------------------------------------------------------------------------------------------------------------


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }

  

  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;




  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  //viewer = simpleVis(cloud);
  //viewer = rgbVis(point_cloud_ptr);
  viewer = rgbVis(rgbcloud);
  viewer->addLine<pcl::PointXYZRGB> (point_1,point_2, "line4");
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }


  return (0);
}




boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
