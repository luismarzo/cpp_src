#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <math.h>

#define PI 3.14159265

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
double polinomical_interpolation(double pp[][2],double xp, int n_points);


int main (int argc, char** argv)
{



	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *rgbcloud) == -1)
  {
    PCL_ERROR ("Couldn't read file .pcd \n");
    return (-1);
  }
    std::cout << "Loaded "
            << rgbcloud->width * rgbcloud->height
            << " data points from .pcd"
            << std::endl;
            
    //Data from uav to interpolate-----------------------------------------
    
    		int n_points,new_points;
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
		
		
		std::cout << "How many NEW points do you want?:";
		std::cin >> new_points;
		double xp[new_points];
		
		std::cout << "Give me the coordinates (x)\n";
		for (int i = 0; i < new_points; i++)
		{
			std::cout << "x(" << i + 1 << "):";
			std::cin >> xp[i];

		}
		double xp_aux;
		double z[new_points];
		for (int i = 0; i < new_points; i++)
		{
		xp_aux=xp[i];
    	        z[i]=polinomical_interpolation(pp,xp_aux,n_points);

    	        }
    //---------------------------------------------------------------------        
  
    uint8_t r1(15), g1(15), b1(255);

      pcl::PointXYZRGB point_1[n_points+new_points];
      for(int i=0;i<n_points+new_points;i++)
      {
      if(i<n_points){
      point_1[i].x = (cos(PI/4)*pp[i][0]);   //el coseno y seno sirve para cambiar los ejes de referencia.
      point_1[i].y = (sin(PI/4)*pp[i][0]);
      point_1[i].z = pp[i][1];
      }
      else{
      point_1[i].x = cos(PI/4)*xp[i-n_points];
      point_1[i].y = sin(PI/4)*xp[i-n_points];
      point_1[i].z = z[i-n_points];
      }

        uint32_t rgb_1 = (static_cast<uint32_t>(r1) << 16 |
              static_cast<uint32_t>(g1) << 8 | static_cast<uint32_t>(b1));
      point_1[i].rgb = *reinterpret_cast<float*>(&rgb_1);


      rgbcloud->points.push_back (point_1[i]);
	
	}



  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = rgbVis(rgbcloud);
  //ordenamos puntos para hacer las lineas
  
  
  int n=n_points+new_points;
  pcl::PointXYZRGB t;
  for (int c = 0 ; c < ( n - 1 ); c++)
  {
    for (int d = 0 ; d < n - c - 1; d++)
    {
      if (point_1[d].x > point_1[d+1].x)
      {
        t         = point_1[d];
        point_1[d]   = point_1[d+1];
        point_1[d+1] = t;
      }
    }
  }

  for(int i=0;i<n_points+new_points-1;i++)
  {
  viewer->addLine<pcl::PointXYZRGB> (point_1[i],point_1[i+1], "line" + i);
  }
 
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
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");
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

/*pcl::PointXYZRGB bubble_sort(pcl::PointXYZRGB list[], int n)
{
  long c, d;
  pcl::PointXYZRGB t;
 
  for (c = 0 ; c < ( n - 1 ); c++)
  {
    for (d = 0 ; d < n - c - 1; d++)
    {
      if (list[d].x > list[d+1].x)
      {
        t         = list[d];
        list[d]   = list[d+1];
        list[d+1] = t;
      }
    }
  }
  for(int i=0;i<5;i++){
  std::cout<<list[i].x<<std::endl;
  }
  return list[];
}*/
