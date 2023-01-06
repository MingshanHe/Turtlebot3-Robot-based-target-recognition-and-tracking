#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

typedef struct
{
    Point2f center;
    int radius;
}target;

class imageSolution
    {
    private:
        /*Global variable*/
        int hmin = 0;
        int hmax = 180;
        int smin = 0;
        int smax = 255;
        int vmin = 0;
        int vmax = 46;
        int g_nStructElementSize = 3;
        int g_nGaussianBlurValue = 6;
    private:
        cv::Mat img;
        cv::Mat img2;
        cv::Mat imghsv;
        cv::Mat mask;
        cv::Mat corrosion;
        cv::Mat gaussian;
        Point2f center;
    private:
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        Mat imgcontours;
        float radius;

    public:
        imageSolution(){};
        ~imageSolution(){};
        Mat image_solution(cv::Mat& img);
        void image_show(cv::Mat& img);
        void image_transfer();
        void image_mask();
        void image_corrosion();
        void image_gaussian();
        target image_contour_extraction();
};
Mat imageSolution::image_solution(cv::Mat& image)
{
    img = image;
    //imageSolution::image_show(img);
    imageSolution::image_transfer();
    imageSolution::image_mask();
    imageSolution::image_corrosion();
    imageSolution::image_gaussian();
    imageSolution::image_contour_extraction();
    return (img2);
}
void imageSolution::image_show(cv::Mat& img)
{
    namedWindow("img_show",cv::WINDOW_NORMAL);
    imshow("img_show",img);
}
void imageSolution::image_transfer()
{
    //namedWindow("img_transfer",cv::WINDOW_NORMAL);
    cvtColor(img,imghsv,COLOR_BGR2HSV);
    //imshow("hsv",imghsv);
}
void imageSolution::image_mask()
{
    //namedWindow("img_mask",cv::WINDOW_NORMAL);
    inRange(imghsv,Scalar(hmin,smin,vmin),Scalar(hmax,smax,vmax),mask);
    //imshow("img_mask",mask);
}
void imageSolution::image_corrosion()
{
    //namedWindow("img_corrosion",cv::WINDOW_NORMAL);
    Mat element = getStructuringElement(MORPH_RECT,Size(2*g_nStructElementSize+1,2*g_nStructElementSize+1),Point(g_nStructElementSize,g_nStructElementSize));
    erode(mask,corrosion,element);
    //imshow("img_corrosion",corrosion);
}
void imageSolution::image_gaussian()
{
    //namedWindow("img_gaussian",cv::WINDOW_NORMAL);
    GaussianBlur(corrosion,gaussian,Size(g_nGaussianBlurValue*2+1,g_nGaussianBlurValue*2+1),0,0);
    //imshow("img_gaussian",gaussian);
}
target imageSolution::image_contour_extraction()
{
    findContours(gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    double maxarea = 0;
    int maxareaidx = 0;
    for (int index = contours.size() - 1; index >= 0; index--)
    {
        double tmparea = fabs(contourArea(contours[index]));
        if (tmparea > maxarea)
        {
            maxarea = tmparea;
            maxareaidx = index;
        }
    }
    minEnclosingCircle(contours[maxareaidx],center,radius);
    img2 = img;
    circle(img2,static_cast<Point>(center), (int)radius, Scalar(255,0,0), 3);
    
    target tar;
    tar.center = center;
    tar.radius = radius;
    return(tar);
}