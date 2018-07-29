#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ransac_geometry.h>
#include <vector>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

typedef struct intrinsic_param{
    int width;
    int height;
    int fx;
    int fy;
    int ppx;
    int ppy;
    void Set(int width_, int height_, int fx_, int fy_, int ppx_, int ppy_)
    {
        width = width_;
        height = height_;
        fx = fx_;
        fy = fy_;
        ppx = ppx_;
        ppy = ppy_;
    }
}intrinsic_param;


// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
void convertDepthToColor(cv::Mat& depth, cv::Mat& color)
{
    const uint8_t nearColor[] = { 255, 0, 0 }, farColor[] = { 20, 40, 255 };
    auto rgbImage = color.ptr<uint8_t>();
    auto depthImage = depth.ptr<uint16_t>();
    auto width = color.cols;
    auto height = color.rows;
    // Produce a cumulative histogram of depth values
    int histogram[256 * 256] = { 1 };
    for (int i = 0; i < width * height; ++i)
    {
        if (auto d = depthImage[i]) ++histogram[d];
    }
    for (int i = 1; i < 256 * 256; i++)
    {
        histogram[i] += histogram[i - 1];
    }
    // Remap the cumulative histogram to the range 0..256
    for (int i = 1; i < 256 * 256; i++)
    {
        histogram[i] = (histogram[i] << 8) / histogram[256 * 256 - 1];
    }
    // Produce RGB image by using the histogram to interpolate between two colors
    auto rgb = rgbImage;
    for (int i = 0; i < width * height; i++)
    {
        if (uint16_t d = depthImage[i]) // For valid depth values (depth > 0)
        {
            auto t = histogram[d]; // Use the histogram entry (in the range of 0..256) to interpolate between nearColor and farColor
            *rgb++ = ((256 - t) * nearColor[0] + t * farColor[0]) >> 8;
            *rgb++ = ((256 - t) * nearColor[1] + t * farColor[1]) >> 8;
            *rgb++ = ((256 - t) * nearColor[2] + t * farColor[2]) >> 8;
        }
        else // Use black pixels for invalid values (depth == 0)
        {
            *rgb++ = 0;
            *rgb++ = 0;
            *rgb++ = 0;
        }                //show color image converted from depth
    }//end for
}
//
inline void _SPTransformFromZImageToZCamera(float one_divide_fx, float one_divide_fy, float u0, float v0, float vin1, float vin2, float vin3, float &vout1, float &vout2, float &vout3)
{
    vout1 = vin3*(vin1 - u0)*one_divide_fx;
    vout2 = vin3*(vin2 - v0)*one_divide_fy;
    vout3 = vin3;
}

void GetPointCloud(unsigned short *psDepth, int width, int height, intrinsic_param intrin, float *pointcloud)
{
    float one_divide_fx = 1.0f / intrin.fx;
    float one_divide_fy = 1.0f / intrin.fy;
    float v1, v2, v3;
    float _v1, _v2, _v3;

    for (size_t y = 0; y < height; y++)
    {
        for (size_t x = 0; x < width; x++)
        {
            int i = y * width + x;
            float d = (float)psDepth[i];

            if (0)//d < 1000.0f || d > 10000.0f)
            {
                pointcloud[3 * i] = 0.0f;
                pointcloud[3 * i + 1] = 0.0f;
                pointcloud[3 * i + 2] = 0.0f;
            }
            else
            {
                _SPTransformFromZImageToZCamera(one_divide_fx, one_divide_fy, intrin.ppx, intrin.ppy,
                    x, y, d, v1, v2, v3);
                pointcloud[3 * i] = 0.0001*v1;
                pointcloud[3 * i + 1] = 0.0001*v2;
                pointcloud[3 * i + 2] = 0.0001*v3;
            }
        }
    }
}

float _PI_ = 3.1415927;
void FilterPointCloudWithNormal(float *pointcloudIn, float *pointcloudOut, int &n, int width, int height)
{
    n = 0;
    for (size_t y = 0; y < height; y += 1)
    {
        for (size_t x = 0; x < width ; x += 1)
        {
            int ii = y * width + x;
            float d = pointcloudIn[3*ii + 2];
            
            if (d > 0.3 && d < 1.2) // too near or too far
            {
               // depthColor.data[3*ii + 1] = 255;
    
                pointcloudOut[3 * n] = pointcloudIn[3 * ii];
                pointcloudOut[3 * n + 1] = pointcloudIn[3 * ii + 1];
                pointcloudOut[3 * n + 2] = pointcloudIn[3 * ii + 2];
                n++;
            }
        }
    }
}

int main()
{
    return 0;
}

float X[3];
float Green[3];
float Y[6];

//
extern "C"
{
    void Run()
    {
        int width = 640;
        int height = 480;

        rs2::pipeline p;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

        rs2::align align_to(RS2_STREAM_DEPTH);

        rs2::pipeline_profile selection = p.start(cfg);
        auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto intrin_ = depth_stream.get_intrinsics();
        intrinsic_param intrin;
        intrin.Set(intrin_.width, intrin_.height, intrin_.fx, intrin_.fy, intrin_.ppx, intrin_.ppy);

        Y[0] = intrin_.width;
        Y[1] = intrin_.height;
        Y[2] = intrin_.fx;
        Y[3] = intrin_.fy;
        Y[4] = intrin_.ppx;
        Y[5] = intrin_.ppy;



        float *pointcloud = new float[width*height*3];
        float *pointcloudOut = new float[width*height*3];

        int NumOfColor =3;
        //1:Red 2:Green 3:Blue
        cv::Mat depthBinary0 = cv::Mat(height, width, CV_8UC1);
        cv::Mat depthBinary1 = cv::Mat(height, width, CV_8UC1);
        cv::Mat depthBinary2 = cv::Mat(height, width, CV_8UC1);
        vector<cv:: Mat> color_data;

        rs2::align align(RS2_STREAM_DEPTH);

        vector<vector<cv::Point>> contours0;
        vector<vector<cv::Point>> contours1;

        vector<cv::Vec4i> hierarchy0;
		vector<cv::Vec4i> hierarchy1;
        while (true)
        {
            // Block program until frames arrive
            rs2::frameset frame = p.wait_for_frames(); 
            rs2::frameset frames = align.process(frame);   
            // Try to get a frame of a depth image

            rs2::frame color_frame = frames.get_color_frame();
            cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
         
            // Get the depth frame's dimensions
            rs2::depth_frame depth = frames.get_depth_frame(); 
            cv::Mat depth_image = cv::Mat(height, width, CV_16UC1);
            cv::Mat depthColor = cv::Mat(depth_image.rows,depth_image.cols, CV_8UC3);
            depth_image.data = (unsigned char*)depth.get_data();




            convertDepthToColor(depth_image, depthColor);

            GetPointCloud((unsigned short*)depth_image.data, width, height, intrin, pointcloud);
            int npoint;
            FilterPointCloudWithNormal(pointcloud, pointcloudOut, npoint, width, height);

            float _g[3] = {0, 0, 1};
            int min_best_total = 5;
            float sigma = 0.1*0.1*0.015;
            int error_code;
            bool bPlaneSuccess = false;
            float A, B, C;
            bPlaneSuccess = ransac_estimation_plane(pointcloudOut, npoint, _g, 0.5, sigma, 4, 150, min_best_total, 2.0, CV_PI/2.0f, A, B, C, error_code);

            for (size_t y = 0; y < height; y++)
            {
                int _i = y*width;
                for (size_t x = 0; x < width; x++)
                {
                    int i = _i + x;
                    int iii = 3*i;
                    float d = (float)pointcloud[3*i+2];

                    depthBinary0.data[i] = 0;
                    depthBinary1.data[i] = 0;
                    depthBinary2.data[i] = 0;

                    if (pointcloud[iii] == 0.0f && pointcloud[iii+1] == 0.0f && pointcloud[iii+2] == 0.0f) // too near or too far
                    {
                        continue;
                    }

                    float fThickness = 0.02f;
          
                    float dis = distance_to_plane_signed(pointcloud[iii], pointcloud[iii+1], pointcloud[iii+2], A, B, C);
                    if (fabs(dis) < fThickness)
                    {
                        depthColor.data[iii + 1] = 255;
                    }
                    else
                    {
                        if (dis < -fThickness)
                        {
                            depthColor.data[iii + 0] = 0;
                            depthColor.data[iii + 1] = 0;
                            depthColor.data[iii + 2] = 0;
                        }
                        else
                        {
                            // Red color
                            if (color_image.data[iii + 2] > 100 && color_image.data[iii+1] < 50 && color_image.data[iii] < 50)
                            {
                                depthBinary0.data[i] = 255;

                            }
                            // Green color
                            if (color_image.data[iii + 1] > 100 && color_image.data[iii] < 100 && color_image.data[iii+2] < 100)
                            {
                                depthBinary1.data[i] = 255;

                            }
                            // Blue color
                            if (color_image.data[iii ] > 100 && color_image.data[iii+1] < 50 && color_image.data[iii+2] < 50)
                            {
                                depthBinary2.data[i] = 255;
                            }

                        }
                    }
                }
            }


            // Collect color data
            color_data.push_back(depthBinary0);
            color_data.push_back(depthBinary1);
            color_data.push_back(depthBinary2);



            cv::findContours(color_data[0], contours0, hierarchy0, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
            int idx = 0;
            double maxArea = -1.0f;
            int maxid = -1;
            if (hierarchy0.size() > 0)
            {
                for(; idx >= 0; idx = hierarchy0[idx][0])
                {
                    double area = cv::contourArea(contours0[idx]);

                    if (area > maxArea)
                    {
                      maxArea = area;
                      maxid = idx;
                    }
                }
        	}


            cv::findContours(color_data[1], contours1, hierarchy1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
            int idx1 = 0;
            double maxArea1 = -1.0f;
            int maxid1 = -1;
            if (hierarchy1.size() > 0)
            {
                for(; idx1 >= 0; idx1 = hierarchy1[idx1][0])
                {
                     double area1 = cv::contourArea(contours1[idx1]);

                    if (area1 > maxArea1)
                    {
                      maxArea1 = area1;
                      maxid1 = idx1;
                    }
                }
        	}
 
 


            vector<vector<Point> > contours_poly0( contours0.size() );
            Rect boundRect0;
            vector<Point2f>center0( contours0.size() );
            vector<float>radius0( contours0.size() );

            vector<vector<Point> > contours_poly1( contours1.size() );
            Rect boundRect1;
            vector<Point2f>center1( contours1.size() );
            vector<float>radius1( contours1.size() );



            if (maxid != -1 )
            {
                approxPolyDP( Mat(contours0[maxid]), contours_poly0[maxid], 3, true );
                boundRect0 = boundingRect( Mat(contours_poly0[maxid]) );
                minEnclosingCircle( (Mat)contours_poly0[maxid], center0[maxid], radius0[maxid] );

                rectangle(depthColor, boundRect0.tl(), boundRect0.br(), Scalar(0, 255, 255), 2, 8, 0 );
                //cout<<boundRect0.tl()<<" "<< boundRect0.br()<<endl;
                int x = (boundRect0.tl().x + boundRect0.br().x)/2;
                int y = (boundRect0.tl().y + boundRect0.br().y)/2;

                X[0] = pointcloud[3*(y*width + x)];
                X[1] = pointcloud[3*(y*width + x)+1];
                X[2] = pointcloud[3*(y*width + x)+2];


            }
            else
            {
                X[0] = 0;
                X[1] = 0;
                X[2] = -1000;


            }

            if ( maxid1 != -1)
            {
 
                approxPolyDP( Mat(contours1[maxid1]), contours_poly1[maxid1], 3, true );
                boundRect1 = boundingRect( Mat(contours_poly1[maxid1]) );
                minEnclosingCircle( (Mat)contours_poly1[maxid1], center1[maxid1], radius1[maxid1] );

                rectangle(depthColor, boundRect1.tl(), boundRect1.br(), Scalar(0, 255, 255), 2, 8, 0 );
                // //cout<<boundRect0.tl()<<" "<< boundRect0.br()<<endl;
                int x = (boundRect1.tl().x + boundRect1.br().x)/2;
                int y = (boundRect1.tl().y + boundRect1.br().y)/2;

                Green[0] = pointcloud[3*(y*width + x)];
                Green[1] = pointcloud[3*(y*width + x)+1];
                Green[2] = pointcloud[3*(y*width + x)+2];

            }
            else
            {

                Green[0] = 0;
                Green[1] = 0;
                Green[2] = -1000;
            }




            cv::namedWindow("colorRed");
            cv::imshow("colorRed", color_data[0]);
            cvWaitKey(1);

            cv::namedWindow("colorGreen");
            cv::imshow("colorGreen", color_data[1]);
            cvWaitKey(1);

            cv::namedWindow("depth2");
            cv::imshow("depth2", depthColor);
            cvWaitKey(1);




        }
    }

    float* GetCentralPoint()
    {
        return X;
    }

    float* GetGreenCentralPoint()
    {
        return Green;
    }


    float* GetIntrinsics()
    {

        return Y;
    }



}