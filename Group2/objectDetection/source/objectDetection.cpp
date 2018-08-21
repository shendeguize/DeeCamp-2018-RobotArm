#include <iostream>
#include <string>
#include <random>
#include <fstream>

#include <librealsense2/rs.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define REALSENSE

#define WIDTH                       640
#define HEIGHT                      480
#define FPS                          30

#define KERNEL_SIZE                   3

#define BLUE_CUBE_THRESH             83 // 80 - 83
#define PINK_CYLI_THRESH             25 // 35 - 40 
#define YELLOW_CUBE_THRESH           60 // 50 - 60
#define GREEN_CUBE_THRESH            60 // 65 - 70

#define CONTOUR_AREA_MIN            700
#define CONTOUR_AREA_MAX           2500
#define BOUND_BOX_SIZE_MIN           15
#define BOUND_BOX_SIZE_MAX           90
#define ENCLOSE_RADIUS_MIN           15
#define ENCLOSE_RADIUS_MAX           30

#define CAT_INDEX_BLUE_CUBE           1 // blue cube
#define CAT_INDEX_PINK_CYLI           2 // pink cylinder
#define CAT_INDEX_YELL_CUBE           3 // yellow cube
#define CAT_INDEX_GREE_CUBE           4 // green cube

#define NON_MAX_SUPP_THRESH         0.6

typedef struct {
    cv::Point2f center;
    float radius;
} Circle;

typedef struct {
    cv::Rect norm_box;
    cv::RotatedRect box;
    Circle circle;
    
    cv::Point2f point;
    float angle;
    int category;

    cv::Scalar color;
} Object;

void realsenseInit(rs2::pipeline pipe);
void getImage(rs2::pipeline pipe, cv::Mat &colorSRC, cv::Mat &depthSrc);
std::string int2String(int n);

void dotsToKernel(cv::InputArray src, void *vecDotVecPtr, void *vecKernelVecPtr);
void readKernelFromFile(std::string _sampleFileDir, void *param);

void distAugment(const cv::Mat src, cv::Mat &dst, const cv::Mat kernel);
void calRectSize(cv::RotatedRect box, float &length, float &width);
float calRectAngle(cv::RotatedRect box);
void drawBoundBox(cv::InputOutputArray mat, cv::RotatedRect box, cv::Scalar color);
void drawNormBoundBox(cv::InputOutputArray mat, cv::Rect normBox, cv::Scalar color);
void drawEncloseCircle(cv::InputOutputArray mat, cv::Point2f center, float radius, cv::Scalar color);

float calIntersection(cv::Rect rect1, cv::Rect rect2);
float calIOU(cv::Rect rect1, cv::Rect rect2);
void nonMaxSuppression(std::vector<Object> &objectVec, float nonMaxThresh);
void drawShape(cv::InputOutputArray mat, std::vector<Object> objectVec);
void printObject(std::vector<Object> objectVec);

int main(int argc, char *argv[])
{
    rs2::pipeline pipe;

    // image
    cv::Mat color, depth;
    cv::Mat depthImg;

    // read kernel vectors from file
    std::string sampleFileDir = "../sample";
    std::string readKernelFlag;
    std::vector<void *> readKernelParamVec;

    std::vector<cv::Mat> blueCubeKerVec;
    std::vector<cv::Mat> pinkCyliKerVec;
    std::vector<cv::Mat> yellowCubeKerVec;
    std::vector<cv::Mat> greenCubeKerVec;

    // compute color filters
    cv::Mat converted;

    cv::Mat blueCubeFilter;
    cv::Mat pinkCyliFilter;
    cv::Mat yellowCubeFilter;
    cv::Mat greenCubeFilter;

    // color filtering
    cv::Mat eroded;
    cv::Mat blueCube;
    cv::Mat pinkCyli;
    cv::Mat yellowCube;
    cv::Mat greenCube;

    cv::RNG randNumGen(99999);
    cv::Scalar blueCubeColor = cv::Scalar(randNumGen.uniform(0, 255), 
                                          randNumGen.uniform(0, 255), 
                                          randNumGen.uniform(0, 255));
    cv::Scalar pinkCyliColor = cv::Scalar(randNumGen.uniform(0, 255), 
                                          randNumGen.uniform(0, 255), 
                                          randNumGen.uniform(0, 255));
    cv::Scalar yellowCubeColor = cv::Scalar(randNumGen.uniform(0, 255), 
                                            randNumGen.uniform(0, 255), 
                                            randNumGen.uniform(0, 255));
    cv::Scalar greenCubeColor = cv::Scalar(randNumGen.uniform(0, 255), 
                                           randNumGen.uniform(0, 255), 
                                           randNumGen.uniform(0, 255));                                        
    std::vector<std::vector<cv::Point2i>> blueCubeContDot;
    std::vector<std::vector<cv::Point2i>> pinkCyliContDot;
    std::vector<std::vector<cv::Point2i>> yellowCubeContDot;
    std::vector<std::vector<cv::Point2i>> greenCubeContDot;

    cv::Rect normMinRect;
    cv::RotatedRect minRect;
    float minRectLen, minRectWid;
    float minRectAngle;
    cv::Point2f encCirCenter;
    float encCirRadius;
    
    cv::Mat destination;
    Object obj;
    std::vector<Object> objectVec;

#ifdef REALSENSE

    /* read kernel vectors from file */
    
    readKernelParamVec.push_back((void *)&blueCubeKerVec);
    readKernelParamVec.push_back((void *)&pinkCyliKerVec);
    readKernelParamVec.push_back((void *)&yellowCubeKerVec);
    readKernelParamVec.push_back((void *)&greenCubeKerVec);
    readKernelFromFile(sampleFileDir, (void *)&readKernelParamVec);
    readKernelParamVec.clear();

    /********************/

    /* compute color filters */

    blueCubeFilter.create(KERNEL_SIZE, KERNEL_SIZE, CV_32SC3);
    pinkCyliFilter.create(KERNEL_SIZE, KERNEL_SIZE, CV_32SC3);
    yellowCubeFilter.create(KERNEL_SIZE, KERNEL_SIZE, CV_32SC3);
    greenCubeFilter.create(KERNEL_SIZE, KERNEL_SIZE, CV_32SC3);
    blueCubeFilter.setTo(0);
    pinkCyliFilter.setTo(0);
    yellowCubeFilter.setTo(0);
    greenCubeFilter.setTo(0);
    blueCubeFilter.setTo(0);

    for (int i = 0; i < blueCubeKerVec.size(); i++) {
        (blueCubeKerVec.at(i)).convertTo(converted, CV_32S);
        blueCubeFilter += converted;
    }
    blueCubeFilter /= blueCubeKerVec.size();
    blueCubeFilter.convertTo(blueCubeFilter, CV_8U);

    for (int i = 0; i < pinkCyliKerVec.size(); i++) {
        (pinkCyliKerVec.at(i)).convertTo(converted, CV_32S);
        pinkCyliFilter += converted;
    }
    pinkCyliFilter /= pinkCyliKerVec.size();
    pinkCyliFilter.convertTo(pinkCyliFilter, CV_8U);

    for (int i = 0; i < yellowCubeKerVec.size(); i++) {
        (yellowCubeKerVec.at(i)).convertTo(converted, CV_32S);
        yellowCubeFilter += converted;
    }
    yellowCubeFilter /= yellowCubeKerVec.size();
    yellowCubeFilter.convertTo(yellowCubeFilter, CV_8U);

    for (int i = 0; i < greenCubeKerVec.size(); i++) {
        (greenCubeKerVec.at(i)).convertTo(converted, CV_32S);
        greenCubeFilter += converted;
    }
    greenCubeFilter /= greenCubeKerVec.size();
    greenCubeFilter.convertTo(greenCubeFilter, CV_8U);

    /********************/

    realsenseInit(pipe);

    while(true){
        /* read image */
        getImage(pipe, color, depth);
        depth.convertTo(depthImg, CV_8UC1);
        destination = color.clone();

        /* color filtering */

        /// Erode
        cv::Mat morElem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::erode(color, eroded, morElem);

        /// Distance Augment
        distAugment(eroded, blueCube, blueCubeFilter);
        distAugment(eroded, pinkCyli, pinkCyliFilter);
        distAugment(eroded, yellowCube, yellowCubeFilter);
        distAugment(eroded, greenCube, greenCubeFilter);


        /// Binary Threshold
        cv::threshold(blueCube, blueCube, BLUE_CUBE_THRESH, 255, cv::THRESH_BINARY_INV);
        cv::threshold(pinkCyli, pinkCyli, PINK_CYLI_THRESH, 255, cv::THRESH_BINARY_INV);
        cv::threshold(yellowCube, yellowCube, YELLOW_CUBE_THRESH, 255, cv::THRESH_BINARY_INV);
        cv::threshold(greenCube, greenCube, GREEN_CUBE_THRESH, 255, cv::THRESH_BINARY_INV);


        /// Find Contours
        cv::findContours(blueCube, blueCubeContDot, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cv::findContours(pinkCyli, pinkCyliContDot, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cv::findContours(yellowCube, yellowCubeContDot, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cv::findContours(greenCube, greenCubeContDot, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        
        /// Contour Processing
        destination = color.clone();

        for (int i = 0; i < blueCubeContDot.size(); i++) {
            if (cv::contourArea(blueCubeContDot[i]) > CONTOUR_AREA_MIN
                && cv::contourArea(blueCubeContDot[i]) < CONTOUR_AREA_MAX) {
                minRect = cv::minAreaRect(blueCubeContDot[i]);
                calRectSize(minRect, minRectLen, minRectWid);
                if(minRectWid > BOUND_BOX_SIZE_MIN && minRectLen < BOUND_BOX_SIZE_MAX) {
                    normMinRect = cv::boundingRect(blueCubeContDot[i]);
                    cv::minEnclosingCircle(blueCubeContDot[i], encCirCenter, encCirRadius);
                    minRectAngle = calRectAngle(minRect);
                    
                    obj.norm_box = normMinRect;
                    obj.box = minRect;
                    obj.circle.center = encCirCenter;
                    obj.circle.radius = encCirRadius;
                    obj.point = minRect.center;
                    obj.angle = minRectAngle;
                    obj.category = CAT_INDEX_BLUE_CUBE;
                    obj.color = blueCubeColor;
                    objectVec.push_back(obj);
                }
            }
        }
        

        for (int i = 0; i < pinkCyliContDot.size(); i++) {
            if (cv::contourArea(pinkCyliContDot[i]) > CONTOUR_AREA_MIN
            && cv::contourArea(pinkCyliContDot[i]) < CONTOUR_AREA_MAX) {
                minRect = cv::minAreaRect(pinkCyliContDot[i]);
                calRectSize(minRect, minRectLen, minRectWid);
                if(minRectWid > BOUND_BOX_SIZE_MIN && minRectLen < BOUND_BOX_SIZE_MAX) {
                    normMinRect = cv::boundingRect(pinkCyliContDot[i]);
                    cv::minEnclosingCircle(pinkCyliContDot[i], encCirCenter, encCirRadius);
                    minRectAngle = 0.0;

                    obj.norm_box = normMinRect;
                    obj.box = minRect;
                    obj.circle.center = encCirCenter;
                    obj.circle.radius = encCirRadius;
                    obj.point = minRect.center;
                    obj.angle = minRectAngle;
                    obj.category = CAT_INDEX_PINK_CYLI;
                    obj.color = pinkCyliColor;
                    objectVec.push_back(obj);
                }
            }
        }

        for (int i = 0; i < yellowCubeContDot.size(); i++) {
            if (cv::contourArea(yellowCubeContDot[i]) > CONTOUR_AREA_MIN
            && cv::contourArea(yellowCubeContDot[i]) < CONTOUR_AREA_MAX) {
                minRect = cv::minAreaRect(yellowCubeContDot[i]);
                calRectSize(minRect, minRectLen, minRectWid);
                if(minRectWid > BOUND_BOX_SIZE_MIN && minRectLen < BOUND_BOX_SIZE_MAX) {
                    normMinRect = cv::boundingRect(yellowCubeContDot[i]);
                    cv::minEnclosingCircle(yellowCubeContDot[i], encCirCenter, encCirRadius);
                    minRectAngle = calRectAngle(minRect);

                    obj.norm_box = normMinRect;
                    obj.box = minRect;
                    obj.circle.center = encCirCenter;
                    obj.circle.radius = encCirRadius;      
                    obj.point = minRect.center;
                    obj.angle = minRectAngle;
                    obj.category = CAT_INDEX_YELL_CUBE;
                    obj.color = yellowCubeColor;
                    objectVec.push_back(obj);
                }
            }
        }

        for (int i = 0; i < greenCubeContDot.size(); i++) {
            if (cv::contourArea(greenCubeContDot[i]) > CONTOUR_AREA_MIN
            && cv::contourArea(greenCubeContDot[i]) < CONTOUR_AREA_MAX) {
                minRect = cv::minAreaRect(greenCubeContDot[i]);
                calRectSize(minRect, minRectLen, minRectWid);
                if(minRectWid > BOUND_BOX_SIZE_MIN && minRectLen < BOUND_BOX_SIZE_MAX) {
                    normMinRect = cv::boundingRect(greenCubeContDot[i]);
                    cv::minEnclosingCircle(greenCubeContDot[i], encCirCenter, encCirRadius);
                    minRectAngle = calRectAngle(minRect);
                    
                    obj.norm_box = normMinRect;
                    obj.box = minRect;         
                    obj.circle.center = encCirCenter;
                    obj.circle.radius = encCirRadius;       
                    obj.point = minRect.center;
                    obj.angle = minRectAngle;
                    obj.category = CAT_INDEX_GREE_CUBE;
                    obj.color = greenCubeColor;
                    objectVec.push_back(obj);
                }
            }
        }

        /// Non-maximum suppression
        nonMaxSuppression(objectVec, NON_MAX_SUPP_THRESH);
        drawShape(destination, objectVec);
        printObject(objectVec);
        objectVec.clear();
        
        /* show result */
        cv::imshow("Color", color);
        cv::imshow("Destination", destination);

        /* take snapshot */
        cv::waitKey(1);
    }

    cv::destroyAllWindows();

#endif // REALSENSE

    return 0;
}

void realsenseInit(rs2::pipeline pipe)
{
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
    pipe.start(cfg);

    return;
}

void getImage(rs2::pipeline pipe, cv::Mat &colorSrc, cv::Mat &depthSrc)
{
    colorSrc.create(HEIGHT, WIDTH, CV_8UC3);
    depthSrc.create(HEIGHT, WIDTH, CV_16UC1);

    rs2::frameset frame = pipe.wait_for_frames();
    rs2::frame colorFrame = frame.get_color_frame();
    rs2::frame depthFrame = frame.get_depth_frame();
    colorSrc.data = (uchar *)colorFrame.get_data();
    depthSrc.data = (uchar *)depthFrame.get_data();

    return;
}

std::string int2String(int n)
{
    int _n, mod;
    std::string str;
    _n = n;
    while(_n){
        mod = _n % 10;
        _n /= 10;
        if(_n == n){
            str = "";
        }
        else{
            str = char(mod + 0x30) + str;// 0x30 is the ASCII code of '0'
        }
    }

    return str;
}

void dotsToKernel(cv::InputArray src, void *vecDotVecPtr, void *vecKernelVecPtr)
{
    cv::Mat _src;
    _src = src.getMat();
    CV_Assert(src.type() == CV_8UC3);
    
    std::vector<std::vector<cv::Point2i>*> *_vecDotVecPtr = (std::vector<std::vector<cv::Point2i>*> *)vecDotVecPtr;
    std::vector<std::vector<cv::Mat>*> *_vecKernelVecPtr = (std::vector<std::vector<cv::Mat>*> *)vecKernelVecPtr;
    
    std::vector<cv::Point2i> *dotVecPtr;
    std::vector<cv::Mat> *kernelVecPtr;
    int x, y;
    cv::Mat srcROI;
    cv::Mat kernel;
    for (int n = 0; n < _vecDotVecPtr->size(); n++) {
        dotVecPtr = _vecDotVecPtr->at(n);
        kernelVecPtr = _vecKernelVecPtr->at(n);
        for (int i = 0; i < dotVecPtr->size(); i++) {
            x = (dotVecPtr->at(i)).x;
            y = (dotVecPtr->at(i)).y;
            std::cout << "dotToKernel (" << x << ", " << y << "): ";
            std::cout <<_src.at<cv::Vec3b>(y, x) << std::endl;
            srcROI = _src(cv::Range(y - 1, y + 2),     // row range - [y - 1, y + 2)
                        cv::Range(x - 1, x + 2));    // col range - [x - 1, x + 2)
            std::cout << srcROI << std::endl;
            kernel = srcROI.clone();
            kernelVecPtr->push_back(kernel);
        }
    }

    return;
}

void readKernelFromFile(std::string _sampleFileDir, void *param)
{
    std::vector<void *> *readFileParamPtr = (std::vector<void *> *)param;
    std::vector<cv::Mat> *blueCubeKerVecPtr;
    std::vector<cv::Mat> *pinkCyliKerVecPtr;
    std::vector<cv::Mat> *yellowCubeKerVecPtr;
    std::vector<cv::Mat> *greenCubeKerVecPtr;

    blueCubeKerVecPtr = (std::vector<cv::Mat> *)(readFileParamPtr->at(0));
    pinkCyliKerVecPtr = (std::vector<cv::Mat> *)(readFileParamPtr->at(1));
    yellowCubeKerVecPtr = (std::vector<cv::Mat> *)(readFileParamPtr->at(2));
    greenCubeKerVecPtr = (std::vector<cv::Mat> *)(readFileParamPtr->at(3));

    std::ifstream blueCubeInFile(_sampleFileDir + "/blueCube.txt", std::ios::in);
    std::ifstream pinkCyliInFile(_sampleFileDir + "/pinkCylinder.txt", std::ios::in);
    std::ifstream yellowCubeInFile(_sampleFileDir + "/yellowCube.txt", std::ios::in);
    std::ifstream greenCubeInFile(_sampleFileDir + "/greenCube.txt", std::ios::in);

    int blue, green, red;
    cv::Vec3b pixelVal;
    cv::Mat kernel(KERNEL_SIZE, KERNEL_SIZE, CV_8UC3);
    while(!blueCubeInFile.eof()){
        for (int i = 0; i < KERNEL_SIZE; i++) {
            for (int j = 0; j < KERNEL_SIZE; j++) {
                blueCubeInFile >> blue >> green >> red;
                pixelVal[0] = uchar(blue);
                pixelVal[1] = uchar(green);
                pixelVal[2] = uchar(red);
                kernel.at<cv::Vec3b>(i, j) = pixelVal;
            }
        }
        blueCubeKerVecPtr->push_back(kernel.clone());
    }
    blueCubeKerVecPtr->pop_back();// there will be a invalid one before end of line.

    while(!pinkCyliInFile.eof()){
        for (int i = 0; i < KERNEL_SIZE; i++) {
            for (int j = 0; j < KERNEL_SIZE; j++) {
                pinkCyliInFile >> blue >> green >> red;
                pixelVal[0] = uchar(blue);
                pixelVal[1] = uchar(green);
                pixelVal[2] = uchar(red);
                kernel.at<cv::Vec3b>(i, j) = pixelVal;
            }
        }
        pinkCyliKerVecPtr->push_back(kernel.clone());
    }
    pinkCyliKerVecPtr->pop_back();

    while(!yellowCubeInFile.eof()){
        for (int i = 0; i < KERNEL_SIZE; i++) {
            for (int j = 0; j < KERNEL_SIZE; j++) {
                yellowCubeInFile >> blue >> green >> red;
                pixelVal[0] = uchar(blue);
                pixelVal[1] = uchar(green);
                pixelVal[2] = uchar(red);
                kernel.at<cv::Vec3b>(i, j) = pixelVal;
            }
        }
        yellowCubeKerVecPtr->push_back(kernel.clone());
    }
    yellowCubeKerVecPtr->pop_back();

    while(!greenCubeInFile.eof()){
        for (int i = 0; i < KERNEL_SIZE; i++) {
            for (int j = 0; j < KERNEL_SIZE; j++) {
                greenCubeInFile >> blue >> green >> red;
                pixelVal[0] = uchar(blue);
                pixelVal[1] = uchar(green);
                pixelVal[2] = uchar(red);
                kernel.at<cv::Vec3b>(i, j) = pixelVal;
            }
        }
        greenCubeKerVecPtr->push_back(kernel.clone());
    }
    greenCubeKerVecPtr->pop_back();

    blueCubeInFile.close();
    pinkCyliInFile.close();
    yellowCubeInFile.close();
    greenCubeInFile.close();

    return;
}

void distAugment(const cv::Mat src, cv::Mat &dst, const cv::Mat kernel)
{
    CV_Assert((src.type() == CV_8UC3));
    CV_Assert((kernel.type() == CV_8UC3) && kernel.size() == cv::Size(KERNEL_SIZE, KERNEL_SIZE));
    dst.create(src.size(), CV_8UC1);

    cv::Mat padding = cv::Mat::zeros(src.rows + 2, src.cols + 2, CV_8UC3);
    cv::Mat paddingROI = padding(cv::Range(1, padding.rows - 1), cv::Range(1, padding.cols - 1));
    src.copyTo(paddingROI);

    cv::Mat res(src.size(), CV_32SC3);
    int cnNum = src.channels();
    const uchar *padPreRowPtr, *padCurRowPtr, *padNexRowPtr;
    const uchar *kerUpRowPtr, *kerMidRowPtr, *kerDwnRowPtr;
    int *resRowPtr;
    kerUpRowPtr = kernel.ptr<uchar>(0);
    kerMidRowPtr = kernel.ptr<uchar>(1);
    kerDwnRowPtr = kernel.ptr<uchar>(2);
    for (int i = 1; i < padding.rows - 1; i++) {
        padPreRowPtr = padding.ptr<uchar>(i - 1);
        padCurRowPtr = padding.ptr<uchar>(i);
        padNexRowPtr = padding.ptr<uchar>(i + 1);
        resRowPtr = res.ptr<int>(i - 1);
        for (int j = cnNum; j < (padding.cols - 1) * cnNum; j++) {
            resRowPtr[j - cnNum] = int( std::abs( padPreRowPtr[j - cnNum] - kerUpRowPtr[j % cnNum] )
                                   + std::abs( padPreRowPtr[j] - kerUpRowPtr[j % cnNum] )
                                   + std::abs( padPreRowPtr[j + cnNum] - kerUpRowPtr[j % cnNum] )
                                   + std::abs( padCurRowPtr[j - cnNum] - kerMidRowPtr[j % cnNum] )
                                   + std::abs( padCurRowPtr[j] - kerMidRowPtr[j % cnNum] )
                                   + std::abs( padCurRowPtr[j + cnNum] - kerMidRowPtr[j % cnNum] )
                                   + std::abs( padNexRowPtr[j - cnNum] - kerDwnRowPtr[j % cnNum] )
                                   + std::abs( padNexRowPtr[j] - kerDwnRowPtr[j % cnNum] )
                                   + std::abs( padNexRowPtr[j + cnNum] - kerDwnRowPtr[j % cnNum] ) );
        }
    }

    cv::Mat resPlane[3];
    cv::Mat dstPlane[3];
    cv::split(res, resPlane);
    dst.convertTo(dst, CV_32SC1);
    dst = 1.0 * resPlane[0] + 1.2 * resPlane[1] + 0.8 * resPlane[2];// 1.0 1.2 .0.8

    double minVal, maxVal;
    cv::minMaxLoc(dst, &minVal, &maxVal);

    double alpha = UCHAR_MAX / (maxVal - minVal);
    double beta = - (UCHAR_MAX * minVal) / (maxVal - minVal);
    dst.convertTo(dst, CV_8UC1, alpha, beta);

    return;
}

void calRectSize(cv::RotatedRect box, float &length, float &width)
{
    cv::Point2f points[4];
    box.points(points);

    float distance;
    distance = cv::norm(points[0] - points[1]);
    length = distance;
    distance = cv::norm(points[1] - points[2]);
    if(distance >= length) {
        width = length;
        length = distance;
    }
    else
        width = distance;

    return;
}

float calRectAngle(cv::RotatedRect box)
{
    cv::Point2f points[4];
    box.points(points);
    cv::Point2f center = box.center;

    cv::Point2f widPoints[2];
    float distance = cv::norm(points[0] - points[1]);
    if(distance >= cv::norm(points[1] - points[2])) {
        widPoints[0] = points[1];
        widPoints[1] = points[2];
    }
    else {
        widPoints[0] = points[0];
        widPoints[1] = points[1];
    }

    cv::Point2f widMid = (widPoints[0] + widPoints[1]) / 2;

    if((widMid.x <= center.x && widMid.y <= center.y)
    || (widMid.x >= center.x && widMid.y >= center.y))
        return (-box.angle + 90.0);
    return -box.angle;
}

void drawBoundBox(cv::InputOutputArray mat, cv::RotatedRect box, cv::Scalar color)
{
    cv::Mat _mat = mat.getMat();
    CV_Assert(_mat.type() == CV_8UC3);
    cv::Point2f points[4];
    box.points(points);


    // draw bounding boxes
    for (int n = 0; n < 4; n++) {
        cv::line(mat, points[n], points[(n + 1) % 4], color, 2, CV_AA);
    }
    // draw bounding box centers
    cv::circle(mat, box.center, 1, color, 2, CV_AA);

    return;
}

void drawNormBoundBox(cv::InputOutputArray mat, cv::Rect normBox, cv::Scalar color)
{
    cv::Mat _mat = mat.getMat();
    CV_Assert(_mat.type() == CV_8UC3);

    // draw normal bounding boxes
    cv::rectangle(_mat, normBox, color, 2, CV_AA);

    // draw normal bounding box centers
    cv::Point2f center;
    center.x = normBox.x + normBox.width / 2;
    center.y = normBox.y + normBox.height / 2;
    cv::circle(_mat, center, 1, color, 2, CV_AA);

    return;
}

void drawEncloseCircle(cv::InputOutputArray mat, cv::Point2f center, float radius, cv::Scalar color)
{
    cv::Mat _mat = mat.getMat();
    CV_Assert(_mat.type() == CV_8UC3);

    // draw enclosing circle in source
    cv::circle(_mat, center, radius, color, 2, CV_AA);
    // draw enclosing circle center in source
    cv::circle(_mat, center, 1, color, 2, CV_AA);

    return;
}

float calIntersection(cv::Rect rect1, cv::Rect rect2)
{
    float x11, y11, x12, y12;
    x11 = rect1.x;
    y11 = rect1.y;
    x12 = rect1.x + rect1.width;
    y12 = rect1.y + rect1.height;
    
    float x21, y21, x22, y22;
    x21 = rect2.x;
    y21 = rect2.y;
    x22 = rect2.x + rect2.width;
    y22 = rect2.y + rect2.height;
    
    float x1, y1, x2, y2;
    x1 = std::max<float>(x11, x21);
    y1 = std::max<float>(y11, y21);
    x2 = std::min<float>(x12, x22);
    y2 = std::min<float>(y12, y22);
    
    if ((x2 - x1 > 0.0) && (y2 - y1 > 0.0)) {
        return (x2 - x1) * (y2 - y1);
    }
    return 0.0;
}

float calIOU(cv::Rect rect1, cv::Rect rect2)
{
    float intersection = calIntersection(rect1, rect2);
    float area1 = rect1.width * rect1.height;
    float area2 = rect2.width * rect2.height;
    return intersection / (area1 + area2 - intersection);
}

void nonMaxSuppression(std::vector<Object> &objectVec, float nonMaxThresh)
{
    std::vector<Object> to_process = objectVec;
    std::vector<Object> processed;
    Object obj1, obj2;

    float iou;    
    float area1, area2;
    int i;
    while(to_process.size() > 0) {
        obj1 = to_process.back();
        to_process.pop_back();
        for (i = 0; i < to_process.size(); i++) {
            obj2 = to_process.at(i);
            // calculate intersection over union
            iou = calIOU(obj1.norm_box, obj2.norm_box);
            // remain smaller one
            if (iou >= nonMaxThresh) {
                area1 = obj1.norm_box.width * obj1.norm_box.height;
                area2 = obj2.norm_box.width * obj2.norm_box.height;
                if (area1 < area2) {
                    to_process.at(i) = obj1;
                }
                // break for loop
                break;
            }
        }
        if(i == to_process.size()) {
            processed.push_back(obj1);
            //drawBoundBox(mat, obj1.box, obj1.color);
            //drawNormBoundBox(mat, obj1.norm_box, cv::Scalar(255, 255, 255));
            //cv::imshow("Mat", mat);
            //cv::waitKey(0);
        }
    }

    objectVec = processed;
    return;
}

void drawShape(cv::InputOutputArray mat, std::vector<Object> objectVec)
{
    cv::Mat _mat = mat.getMat();
    CV_Assert(_mat.type() == CV_8UC3);

    Object obj;
    for (int i = 0; i < objectVec.size(); i++) {
        obj = objectVec.at(i);
        switch(obj.category) {
            case CAT_INDEX_BLUE_CUBE:
            case CAT_INDEX_YELL_CUBE:
            case CAT_INDEX_GREE_CUBE:
                drawBoundBox(_mat, obj.box, obj.color);
                break;
            case CAT_INDEX_PINK_CYLI:
                drawEncloseCircle(_mat, obj.circle.center, obj.circle.radius, obj.color);
                break;
        }
    }

    return;
}

void printObject(std::vector<Object> objectVec)
{
    Object obj;
    
    for (int i = 0; i < objectVec.size(); i++) {
        obj = objectVec.at(i);
        std::cout << obj.point.x << " " << obj.point.y << " " << obj.angle << " " << obj.category << std::endl;
    }

    return;
}