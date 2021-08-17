#ifndef GEOMERTY_H
#define GEOMERTY_H

// #include <ros/ros.h>
// #include <ros/package.h>
// #include <iostream>
// #include <vector>
// #include <map>
// #include <stack>
// #include <algorithm>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <cmath>
#include <map>
#include <stack>
#include <algorithm>
#include <std_msgs/String.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

// struct Points
// {
//     double x;    // x座標
//     double y;    // y座標
//     double z;    // z座標（預設為0，如果需要三維點則給z賦值）

//     Points(double a = 0, double b = 0, double c = 0) { x = a; y = b; z = c; } // 建構函式
// };

struct Line
{
    Point3f s;    // 起點
    Point3f e;    // 終點
    bool is_seg; // 是否是線段

    Line() {};    // 預設建構函式
    Line(Point3f a, Point3f b, bool _is_seg = true) { s = a; e = b; is_seg = _is_seg; }    // 建構函式(預設是線段)
};



class Computational_geometry 
{
    public:
        Computational_geometry()
        {
            
        };
        ~Computational_geometry(){};
        static Computational_geometry* getInstance();
        static void deleteInstance();
        bool equal(const Point3f& lhs, const Point3f& rhs);
        bool isSegIntersect(const Line& l1, const Line& l2,Point3f& inter_p);
        void boxOfPolygon(const vector<Point3i>& polygon, Point3f& down_left, Point3f& up_right);
        bool isPointInPolygon(const vector<Point3i>& polygon, const Point3i& p);
        bool isponl(const Point3f& p, const Line& l);
        double length(const Point3f& vec);
        double Cos(const Point3f& vec1, const Point3f& vec2);
        double dotMultiply(const Point3f& vec1, const Point3f& vec2);
        Point3f add(const Point3f& lhs, const Point3f& rhs);
        Point3f mul(const Point3f& p, double ratio);
        Point3f div(const Point3f& p, double ratio);
        Point3f multiply(const Point3f& vec1, const Point3f& vec2);
        Point3f normalize(const Point3f& vec);
        Point3f sub(const Point3f& lhs, const Point3f& rhs);
    private:
        static Computational_geometry *m_pInstance;
};

class Computational_geometryInstance : public Computational_geometry
{
public:
    Computational_geometryInstance() : Computational_geometry(){}
    ~Computational_geometryInstance(){}
    static Computational_geometryInstance* getInstance();
    static void deleteInstance();
private:
    static Computational_geometryInstance* m_pInstance;
};


#endif // GEOMERTY_H