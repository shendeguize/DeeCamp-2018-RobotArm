#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <stdio.h>
#include <string>
#include <iostream>

#define WIDTH           640
#define HEIGHT          480
#define FPS             30

using namespace cv;
using namespace pcl;
using namespace std;

typedef PointXYZRGB PointT;

const double g_camera_factor = 1000;
const double g_camera_cx = 329.2113;
const double g_camera_cy = 229.1156;
const double g_camera_fx = 615.7779;
const double g_camera_fy = 616.0148;
string objects(string frame_data, rs2::pipeline& pipe, visualization::PCLVisualizer& viewer);
void process(Mat& bgr_process);
void on_MouseHandle(int event, int x, int y, int flags, void* param);

int main()
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
    pipe.start(cfg);

    // Mouse event
    //namedWindow("record_point_window", WINDOW_NORMAL);
    setMouseCallback("record_point_window",on_MouseHandle);

    // PCL viewer
    visualization::PCLVisualizer viewer("Cloud viewer");
    viewer.setCameraPosition(0,0,-3.0,0,-1,0);
    viewer.addCoordinateSystem(0.3);

    while(true)
    {
        string objects_data;
        objects_data = objects(objects_data, pipe, viewer);
        cout << objects_data << "\n" << endl;
    }
    return 0;
}

string objects(string objects_data, rs2::pipeline& pipe, visualization::PCLVisualizer& viewer)
{
    for(int a=0; a<1; a++)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth_raw = data.get_depth_frame();
        rs2::frame color_raw = data.get_color_frame();
        //const int w = depth.as<rs2::video_frame>().get_width();
        //const int h = depth.as<rs2::video_frame>().get_height();
        while(!depth_raw||!color_raw)
        {
            data = pipe.wait_for_frames();
            rs2::frame depth_raw = data.get_depth_frame();
            rs2::frame color_raw = data.get_color_frame();
        }
        Mat depth(Size(WIDTH, HEIGHT), CV_16U, (void*)depth_raw.get_data(), Mat::AUTO_STEP);
        Mat bgr(Size(WIDTH, HEIGHT), CV_8UC3, (void*)color_raw.get_data(), Mat::AUTO_STEP);
        Rect rect_depth(WIDTH/8, HEIGHT/25+HEIGHT/8, WIDTH-WIDTH/32-WIDTH/8, HEIGHT-HEIGHT/25-HEIGHT/8);
        Rect rect_color(WIDTH/32+WIDTH/8, 0+HEIGHT/8, WIDTH-WIDTH/32-WIDTH/8, HEIGHT-HEIGHT/25-HEIGHT/8);
        bgr = bgr(rect_color);
        depth = depth(rect_depth);

        PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>);
        for(int m=0; m<depth.rows; m++)
        {
            for(int n=0; n<depth.cols; n++)
            {
                ushort d = (ushort)depth.at<ushort>(m, n);
                if(d == 0)
                {
                    continue;
                }
                PointT p;

                p.z = (double)d / g_camera_factor;
                p.x = (n - g_camera_cx) * p.z / g_camera_fx;
                p.y = (m - g_camera_cy) * p.z / g_camera_fy;

                p.b = bgr.ptr<uchar>(m)[n*3];
                p.g = bgr.ptr<uchar>(m)[n*3 + 1];
                p.r = bgr.ptr<uchar>(m)[n*3 + 2];
                cloud->points.push_back(p);
            }
        }
        cloud->height = 1;
        cloud->width = cloud->points.size();
        cloud->is_dense = false;
        if(cloud->points.size()<10) continue;

        PassThrough<PointT> vg;
        PointCloud<PointT>::Ptr cloud_filtered(new PointCloud<PointT>);
        vg.setInputCloud(cloud);
        //vg.setLeafSize(0.001f, 0.001f, 0.001f);
        vg.setFilterFieldName ("z");
        vg.setFilterLimits (0.45, 0.6);
        vg.filter(*cloud_filtered);
        if(cloud_filtered->points.size()<10) continue;

        SACSegmentation<PointT> seg;
        PointIndices::Ptr inliers(new  PointIndices);
        ModelCoefficients::Ptr coefficients(new ModelCoefficients);
        seg.setOptimizeCoefficients(new ModelCoefficients);
        seg.setModelType(SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.008);

        //int nr_points = (int)cloud_filtered->points.size();
        //while(cloud_filtered->points.size() > 0.1*nr_points)
        {
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if(inliers->indices.size() == 0)
            {
                break;
            }
            ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud_filtered);
        }
        viewer.removePointCloud();
        viewer.addPointCloud(cloud_filtered);
        viewer.spinOnce(20);

        Mat bgr_back(Size(depth.cols, depth.rows), CV_8UC3, Scalar(0, 0, 0));
        unsigned long cloudSize = cloud_filtered->points.size();
        for(unsigned long i=0; i<cloudSize; i++)
        {
            int m, n;
            double x, y, z;
            x = cloud_filtered->points[i].x;
            y = cloud_filtered->points[i].y;
            z = cloud_filtered->points[i].z;
            n = x * g_camera_fx / z + g_camera_cx;
            m = y * g_camera_fy / z + g_camera_cy;
            bgr_back.at<Vec3b>(m, n)[0] = bgr.at<Vec3b>(m, n)[0];
            bgr_back.at<Vec3b>(m, n)[1] = bgr.at<Vec3b>(m, n)[1];
            bgr_back.at<Vec3b>(m, n)[2] = bgr.at<Vec3b>(m, n)[2];
            if(m<1) m=1;
            if(n<1) n=1;
            for(int j=0; j<3; j++)
            {
                bgr_back.at<Vec3b>(m-1, n-1)[j] = bgr.at<Vec3b>(m-1, n-1)[j];
                bgr_back.at<Vec3b>(m, n-1)[j] = bgr.at<Vec3b>(m, n-1)[j];
                bgr_back.at<Vec3b>(m-1, n)[j] = bgr.at<Vec3b>(m-1, n)[j];
            }
        }
        //process(bgr_back);
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        //erode(bgr_back, bgr_back, element);
        dilate(bgr_back, bgr_back, element);
        imshow("RAN_SAC", bgr_back);
        cvtColor(bgr_back, bgr_back, COLOR_BGR2GRAY);
        threshold(bgr_back, bgr_back, 30, 200.0, THRESH_BINARY);
        GaussianBlur(bgr_back, bgr_back, Size(3, 3), 2, 2);

        //Canny(bgr_back, bgr_back, 150, 100, 3);
        //imshow("Canny", bgr_back);

        std::vector<std::vector<Point>> contours ;
        //Without contours inside
        findContours(bgr_back , contours ,
            RETR_EXTERNAL , CHAIN_APPROX_NONE) ;
        Mat result(bgr_back.size() , CV_8U , Scalar(0)) ;
        drawContours(result , contours ,
            -1 , Scalar(255) , 2) ;
        //// Include contours inside
        //std::vector<std::vector<Point>> allContours ;
        //Mat allContoursResult(image.size() , CV_8U , Scalar(255)) ;
        //findContours(image , allContours ,
        //    RETR_LIST , CHAIN_APPROX_NONE) ;
        //drawContours(allContoursResult , allContours ,-1 ,
        //    Scalar(0) , 2) ;
        //imshow("allContours" , allContoursResult) ;
        ////Obtain hierarchy
        //std::vector<Vec4i> hierarchy ;
        //findContours(image , contours , hierarchy , CV_RETR_TREE ,
        //    CHAIN_APPROX_NONE) ;
        std::vector<std::vector<Point>>::const_iterator itc = contours.begin();
        int num_objects = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            // Minimum size
            if(contourArea(contours[i])<60) continue;
            // Find a circle to enclose
            Point2f center;
            float radius = 0;
            minEnclosingCircle(contours[i], center, radius);
            if(radius<5||radius>100) continue;
            // draw center
            circle(result,
            // position of mass center converted to integer
            center,2,Scalar(255),2); // draw black dot
            int m = ceil(center.x);
            int n = ceil(center.y);
            if(m<0) m=0;
            if(n<0) n=0;
            if(m>depth.rows) m=depth.rows-1;
            if(n>depth.cols) n=depth.cols-1;
            ushort d = depth.at<ushort>(m, n);
            double z = d / g_camera_factor;
            double x = (n - g_camera_cx) * z / g_camera_fx;
            double y = (m - g_camera_cy) * z / g_camera_fy;
            ostringstream out;
            //printf("(%f, %f, %f)\n", x, y, z);
            out<<"("<<x<<","<<y<<","<<z<<")";
            objects_data+=out.str();
            num_objects++;
        }
        printf("num_objects:%d\n", num_objects);
        imshow("resultImage" , result) ;

        /*vector<Vec3f> circles;
        HoughCircles(bgr_back, circles, HOUGH_GRADIENT, 1.5, bgr_back.rows / 4, 200, 250, 0, 0);
        for(size_t i=0; i<circles.size(); i++)
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int m = cvRound(circles[0][0]);
            int n = cvRound(circles[0][1]);
            ushort d = depth.at<ushort>(m, n);
            double z = (double)d / g_camera_factor;
            double x = (n - g_camera_cx) * z / g_camera_fx;
            double y = (m - g_camera_cy) * z / g_camera_fy;
            printf("%f, %f, %f\n", x, y, z);
            int radius = cvRound(circles[i][2]);
            circle(bgr, center, 3, Scalar(0,255,0), -1, 8, 0);
            circle(bgr, center, radius, Scalar(0, 0,255), 2, 8, 0);
        }*/
        waitKey(1);
    }

    return objects_data;
}

inline void process(Mat& bgr_process)
{
    for(int i=0; i<bgr_process.rows; i++)
    {
        for(int j=0; j<bgr_process.cols; j++)
        {
            unsigned char g = bgr_process.at<Vec3b>(i, j)[1];
            unsigned char b = bgr_process.at<Vec3b>(i, j)[0];
            unsigned char r = bgr_process.at<Vec3b>(i, j)[2];
            if( g>b && r>b)
            {
                if(g>=240 && r>220 && b<230){}
                else if(g>=210 && g<=240 && r>165 && b<175){}
                else if(g>=190 && g<=210 && r>178 && r<230 && b<150){}
                else if(g>=180 && g<=190 && r>165 && r<200 && b<130){}
                else if(g>=170 && g<=180 && r>155 && r<190 && b<115){}
                else if(g>=140 && g<=170 && r>130 && r<185 && b<110){}
                else if(g>=130 && g<=140 && r>125 && r<145 && b<105){}
                else if(g>=120 && g<=130 && r>110 && r<150 && b< 90){}
                else if(g>=100 && g<=120 && r>100 && r<130 && b< 75){}
                else if(g>= 80 && g<=100 && r> 85 && r<125 && b< 40){}
                else if(g>= 70 && g<= 80 && r> 70 && r< 95 && b< 40){}
                else if(g>= 60 && g<= 70 && r> 55 && r< 75 && b< 40){}
                else if(g>= 50 && g<= 60 && r> 45 && r< 60 && b< 30){}
                else if(g>= 40 && g<= 50 && r> 40 && r< 55 && b< 20){}
                else
                {
                    bgr_process.at<Vec3b>(i, j)[0] = 0;
                    bgr_process.at<Vec3b>(i, j)[1] = 0;
                    bgr_process.at<Vec3b>(i, j)[2] = 0;
                }
            }
            else
            {
                bgr_process.at<Vec3b>(i, j)[0] = 0;
                bgr_process.at<Vec3b>(i, j)[1] = 0;
                bgr_process.at<Vec3b>(i, j)[2] = 0;
            }
        }
    }
}

void on_MouseHandle(int event, int x, int y, int flags, void* param)
{
    Point pt;
    switch(event)
    {
    case EVENT_LBUTTONDOWN:
        {
            pt = Point(x, y);
            cout << "  " << pt.x << " " << pt.y << endl;
            int m = x, n = y;
            double x = (n - g_camera_cx) * 0.56 / g_camera_fx;
            double y = (m - g_camera_cy) * 0.56 / g_camera_fy;
            printf("%f, %f\n", x, y);
        }
        break;
    }
}























