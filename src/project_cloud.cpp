#include <iostream>
#include <stdio.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/filter.h>

#include <pcl/io/obj_io.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "cvaux.h"

#include <pcl/io/io.h>

#define PT PointXYZRGB

using namespace pcl;
using namespace cv;
using namespace std;

void createProjection(PointCloud<PT> cloud, const Point3f location, const Point3f direction)
{
    float f = 4.0;

    int w = 1000;
    int h = 1000;

    Mat image(w, h, CV_8UC3);

    image = Scalar(0, 0, 0);

    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(cloud, cloud, mapping);

    for (PointCloud<PT>::iterator iter = cloud.begin(); iter != cloud.end(); ++iter)
    {
        float x1 = iter->x - location.x;
        float x2 = iter->y - location.y;
        float x3 = iter->z - location.z;

        int y1 = (f * x1 / x3)*50;
        int y2 = (f * x2 / x3)*50;

        //cout << "x1: " << x1 << "\tx2: " << x2 << "\tx3: " << x3 << "\ty1: " << y1 << "\ty2:" << y2 << endl;

        // cout << "editing" << endl;
        if (y1 >= w/2 || y1 < -w/2 || y2 >= h/2 || y2 < -h/2)// || x3 < 0)
        {
            //cout << "y1: " << y1 << "y2: " << y2 << endl;
            continue;
        }
        try {
            Point3_<uchar>* p = image.ptr<Point3_<uchar> >(y2+ w/2,y1+h/2);
            p->x = (int) iter->b; //B
            p->y = (int) iter->g; //G
            p->z = (int) iter->r; //R
           // cout << "[GOOD POINT] " << "y1: " << y1 << "y2: " << y2 << endl;

        } catch (...) {
            ;
        }
        // cout << "done" << endl;
    }

   // namedWindow("1");
   // imshow("1", image);
    imwrite("img.jpg", image);
}

int main(int argc, char** argv)
{
	PointCloud<PT>::Ptr cloud (new pcl::PointCloud<PT>);

    io::loadPCDFile<PT>(argv[1], *cloud);

    Point3f location(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
    Point3f orientation(1,1,1);

    createProjection(*cloud, location, orientation);

    /*
	pcl::PolygonMesh mesh;

	pcl::io::loadPolygonFileOBJ("scan.obj", mesh);
 */
    // Start the visualizer
    pcl::visualization::PCLVisualizer p ("Mesh");
    p.addCoordinateSystem (0.1);
    p.addPointCloud(cloud,"mesh");
    p.spin ();

    return 0; 
} 