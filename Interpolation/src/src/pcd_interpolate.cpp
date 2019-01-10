#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
double polinomical_interpolation(double pp[][2],double xp, int n_points);

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/luis/cpp_src/Interpolation/src/maps/strimona_bridge_spam2.pcd", *rgbcloud) == -1)
  {
    PCL_ERROR ("Couldn't read file .pcd \n");
    return (-1);
  }
    std::cout << "Loaded "
            << rgbcloud->width * rgbcloud->height
            << " data points from .pcd"
            << std::endl;
            
    //Data from uav to interpolate-----------------------------------------
    
    		int n_points;
		double xp;
		std::cout << "How many points are you going to introduce?:";
		std::cin >> n_points;
		double pp[n_points][2];

		std::cout << "Give me the (x,z) points:\n";
		for (int i = 0; i < n_points; i++)
		{
			std::cout << "x(" << i + 1 << "):";
			std::cin >> pp[i][0];
			std::cout << "z(" << i + 1 << "):";
			std::cin >> pp[i][1];
		}
		
		std::cout << "Give me the coordinate (x)\n";
		std::cout << "x:";
		std::cin >> xp;
    		double z=polinomical_interpolation(pp,xp,n_points);
    //---------------------------------------------------------------------        
  
    uint8_t r1(15), g1(15), b1(255);
      pcl::PointXYZRGB point_1,point_2,point_3,point_4;
      point_1.x = pp[0][0];
      point_1.y = 0;
      point_1.z = pp[0][1];
      
      point_2.x = pp[1][0];;
      point_2.y = 0;
      point_2.z = pp[1][1];
      
      point_3.x = pp[2][0];
      point_3.y = 0;
      point_3.z = pp[2][1];
           
      point_4.x = xp;
      point_4.y = 0;
      point_4.z = z;
      
  
        uint32_t rgb_1 = (static_cast<uint32_t>(r1) << 16 |
              static_cast<uint32_t>(g1) << 8 | static_cast<uint32_t>(b1));
      point_1.rgb = *reinterpret_cast<float*>(&rgb_1);
      point_2.rgb = *reinterpret_cast<float*>(&rgb_1);
      point_3.rgb = *reinterpret_cast<float*>(&rgb_1);
      point_4.rgb = *reinterpret_cast<float*>(&rgb_1);

      rgbcloud->points.push_back (point_1);
      rgbcloud->points.push_back (point_2);
      rgbcloud->points.push_back (point_3);
      rgbcloud->points.push_back (point_4);




  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = rgbVis(rgbcloud);
  viewer->addLine<pcl::PointXYZRGB> (point_1,point_2, "line4");
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }


  return (0);
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

double polinomical_interpolation(double pp[][2],double xp, int n_points){
	
	

			int cont=1;	
			double y=1;
			Eigen::MatrixXd m(n_points, n_points);
			Eigen::VectorXd k(n_points, 1);
			for(int i=0;i<n_points;i++){
				m(i,0)=1;
				k(i,0) = pp[i][1];
				
				}
				
			for(int i=1;i<n_points;i++){
				for(int j=0;j<n_points;j++){
				
					for (int k = 0; k < cont; k++)
					{
						y = y*(pp[j][0]);

					}
					
					m(j,i)=y;
					y=1;
				}
				cont++;
				
				
			
			}
	
			
			Eigen::VectorXd x = m.colPivHouseholderQr().solve(k);
			int cnt = 1;
			double sol=0;
			y=1;

			
			for (int i = 0; i < n_points; i++)
			{
				if (i == 0)
				{
					sol = sol + x[i];
				}
				else
				{
					for (int j = 0; j < cnt; j++)
					{
						y=y*xp;
					}
					cnt++;
					sol = sol + x[i] * y;
					y = 1;
				}
			}

			std::cout << "The solution z is:\n"
					  << sol << std::endl;
			return sol;
	
}
