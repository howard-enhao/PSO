#include <strategy/computational_geometry.h>

Computational_geometryInstance* Computational_geometryInstance::m_pInstance;

Computational_geometryInstance* Computational_geometryInstance::getInstance()
{
    if(!m_pInstance)m_pInstance = new Computational_geometryInstance();
    return m_pInstance;
}

void Computational_geometryInstance::deleteInstance()
{
    if(m_pInstance)
    {
        delete m_pInstance;
        m_pInstance = NULL;
    }
}

/*  點相加*/
Point3f Computational_geometry::add(const Point3f& lhs, const Point3f& rhs)
{
    Point3f res;

    res.x = lhs.x + rhs.x;
    res.y = lhs.y + rhs.y;
    res.z = lhs.z + rhs.z;

    return res;
}

/*  點相減*/
Point3f Computational_geometry::sub(const Point3f& lhs, const Point3f& rhs)
{
    Point3f res;

    res.x = lhs.x - rhs.x;
    res.y = lhs.y - rhs.y;
    res.z = lhs.z - rhs.z;

    return res;
}

/*  向量相乘*/
Point3f Computational_geometry::mul(const Point3f& p, double ratio)
{
    Point3f res;

    res.x = p.x * ratio;
    res.y = p.y * ratio;
    res.z = p.z * ratio;

    return res;
}

/*  向量相除*/
Point3f Computational_geometry::div(const Point3f& p, double ratio)
{
    Point3f res;
    
    res.x = p.x / ratio;
    res.y = p.y / ratio;
    res.z = p.z / ratio;

    return res;
}

/*  點到線ㄉ投影點
    p：點
    l：線*/
Point3f Computational_geometry::ptolProjection(const Point3f& p, const Line& l)
{
    Point3f line_vec = sub(l.e, l.s);
    Point3f point_vec = sub(p, l.s);
    Point3f unit_line_vec = normalize(line_vec);

    // 投影長度
    double project_len = dotMultiply(point_vec, unit_line_vec);

    // 投影點
    Point3f project_p = add(l.s,mul(unit_line_vec, project_len));

    return project_p;
}

/*  向量標準化
    vec：向量*/
Point3f Computational_geometry::normalize(const Point3f& vec)
{
    Point3f res;

    res = div(vec, length(vec));

    return res;
}

/*  向量叉乘
    vec1：向量1
    vec2：向量2*/
Point3f Computational_geometry::multiply(const Point3f& vec1, const Point3f& vec2)
{
    Point3f result;

    result.x = vec1.y * vec2.z - vec2.y * vec1.z;
    result.y = vec1.z * vec2.x - vec2.z * vec1.x;
    result.z = vec1.x * vec2.y - vec2.x * vec1.y;

    return result;
}

/*  向量點乘
    vec1：向量1
    vec2：向量2*/
double Computational_geometry::dotMultiply(const Point3f& vec1, const Point3f& vec2)
{
    return(vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z);
}

/*  向量長度
    vec：向量*/
double Computational_geometry::length(const Point3f& vec)
{
    return (sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2)));
}

/*  點到線ㄉ距離
    p：點
    l：線*/
double Computational_geometry::ptolDistance(const Point3i& p, const Line& l)
{
    Point3i line_vec = sub(l.e,l.s);
    Point3i point_vec = sub(p, l.s);
    // cout<<l.e<<","<<l.s<<","<<p<<endl;
    // cout<<line_vec<<","<<point_vec<<endl;
    // 計算點線上段投影長度
    double project_len = dotMultiply(line_vec, point_vec) / length(line_vec);

    // 點的距離(勾股定理)
    double distance = sqrt(pow(length(point_vec), 2) - pow(project_len, 2));
    // printf("sqrt(%f-%f,2)\n", pow(length(point_vec), 2), pow(project_len, 2));
    // printf("project_len= %f, distance=%f\n", project_len, distance);
    return distance;
}

double Computational_geometry::Cos(const Point3f& vec1, const Point3f& vec2)
{
    Point3f unit_vec1 = normalize(vec1);
    Point3f unit_vec2 = normalize(vec2);

    return dotMultiply(unit_vec1, unit_vec2);
}

/*  判斷點是否相等*/
bool Computational_geometry::equal(const Point3f& lhs, const Point3f& rhs)
{
    return(lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

/*  點是否在線上
    線分為直線和線段，直線表示的是直線是否經過點
    p：點
    l：線段or線*/
bool Computational_geometry::isponl(const Point3f& p, const Line& l)
{
    Point3f line_vec = sub(l.e, l.s);
    Point3f point_vec1 = sub(p, l.s);
    Point3f point_vec2 = sub(p, l.e);

    Point3f mul_vec = multiply(line_vec, point_vec1);
    double dot = dotMultiply(point_vec1, point_vec2);
    // 點是否線上段上
    if (l.is_seg)
    {
        if (equal(p,l.s) || equal(p,l.e))
            return true;
        return (0.0 == length(mul_vec) && dot < 0.0);
        
    }
    // 點是否在直線上
    else
    {
        return (0.0 == length(mul_vec));
    }
}

/*  判斷線段是否相交
    如果線段的端點重合或者某個線段端點在另一個線段上也算相交
    線段判斷是否相交，如果是直線則相當於判斷是否平行
    l1：線段1
    l2：線段2
    inter_p：如果相交就返回交點*/
bool Computational_geometry::isSegIntersect(const Line& l1, const Line& l2,Point3f& inter_p)
{
    Point3f line1 = sub(l1.e, l1.s);
    Point3f line2 = sub(l2.e, l2.s);
    Point3f norm1 = normalize(line1);
    Point3f norm2 = normalize(line2);
    
    // 線段相交
    if (l1.is_seg)
    {
        // 端點線上段上
        if (isponl(l1.s, l2))
        {
            inter_p = l1.s;
            return true;
        }
        if (isponl(l1.e, l2))
        {
            inter_p = l1.e;
            return true;
        }
        if (isponl(l2.s, l1))
        {
            inter_p = l2.s;
            return true;
        }
        if (isponl(l2.e, l1))
        {
            inter_p = l2.e;
            return true;
        }
        // 判斷線段是否相互跨立
        double dot1 = dotMultiply(multiply(sub(l2.s, l1.s), line1), multiply(sub(l2.e, l1.s), line1));
        double dot2 = dotMultiply(multiply(sub(l1.s, l2.s), line2), multiply(sub(l1.e, l2.s), line2));
        if (dot1 < 0.0 && dot2 < 0.0)
        {
            float t1 = length(multiply(sub(l1.s, l2.s), norm2)) / length(multiply(norm2, norm1));
            double t2 = length(multiply(sub(l2.s, l1.s), norm1)) / length(multiply(norm1, norm2));
            inter_p = add(l1.s, mul(norm1, t1));
            return true;
        }
        else
        {
            return false;
        }

    }
    // 直線相交
    else
    {
        if (Cos(line1, line2) == 1.0)
            return false;

        double t1 = length(multiply(sub(l1.s, l2.s), norm2)) / length(multiply(norm2, norm1));
        double t2 = length(multiply(sub(l2.s, l1.s), norm1)) / length(multiply(norm1, norm2));

        inter_p = add(l1.s, mul(norm1, t1));
        return true;
    }
}

/*  獲取多邊形左下跟右上的兩點
    polygon：多邊形的點
    down_left：左下點
    up_right：右上點*/
void Computational_geometry::boxOfPolygon(const vector<Point3i>& polygon, Point3f& down_left, Point3f& up_right)
{
    double max_x = double(INT_MIN), min_x = double(INT_MAX);
    double max_y = double(INT_MIN), min_y = double(INT_MAX);

    for (auto c : polygon)
    {
        max_x = (c.x > max_x) ? c.x : max_x;
        min_x = (c.x < min_x) ? c.x : min_x;
        max_y = (c.y > max_y) ? c.y : max_y;
        min_y = (c.y < min_y) ? c.y : min_y;
    }

    down_left = Point3f(min_x, min_y, 0);
    up_right = Point3f(max_x, max_y, 0);
}

/*  判斷點是否在多邊形內部
    polygon：多邊形的點
    p：要判斷的點*/
bool Computational_geometry::isPointInPolygon(const vector<Point3i>& polygon, const Point3i& p)
{
    Point3f down_left, up_right, out_p;
    boxOfPolygon(polygon, down_left, up_right);

    // 位於多邊形外部一點
    if(down_left.x<3 || down_left.y<3)
        out_p = add(up_right, Point3f(3.0, 4.0, 0.0));
    else
        out_p = sub(down_left, Point3f(2.0, 3.0, 0.0));
    
    int cnt(0);
    Line p_line(p, out_p, true);
    Mat Contours=Mat::zeros(cv::Size(320,240),CV_8UC3);
    Contours.at<Vec3b>(Point(p.x, p.y))[0]=255;
    Contours.at<Vec3b>(Point(p.x, p.y))[1]=255;
    Contours.at<Vec3b>(Point(p.x, p.y))[2]=255;
    Contours.at<Vec3b>(Point(out_p.x, out_p.y))[0]=0;
    Contours.at<Vec3b>(Point(out_p.x, out_p.y))[1]=255;
    Contours.at<Vec3b>(Point(out_p.x, out_p.y))[2]=0;

    for (int i = 0; i < polygon.size(); ++i)
    {
        Point3f s = polygon[i];
        Point3f e = polygon[(i + 1) % polygon.size()];
        Line seg(s, e, true);
        Point3f inter_p;
        line(Contours, Point(polygon[i].x, polygon[i].y), Point(polygon[(i + 1) % polygon.size()].x, polygon[(i + 1) % polygon.size()].y), Scalar(89, 90, 90), 1);
        if (isSegIntersect(p_line, seg, inter_p))
        {
            Point P=Point(inter_p.x,inter_p.y);
            cnt++;
        }
    }
    // cout<<"cnt = "<<cnt<<endl;  //cnt奇數為點在obs內，偶數則反
    // imshow("inter_p",Contours);
    // waitKey(1);

    return (cnt % 2 == 1);
}

/*  線段與圓ㄉ關係
    c：圓心
    radius：半徑
    l：判斷的線段
    return：0=圓內，1=與圓香蕉，2=圓外*/
int Computational_geometry::segToCircle(const Point3f& c, double radius, const Line& l)
{
    double ctol_d = ptolDistance(c, l);
    // printf("ctol = %f, radius = %f\n", ctol_d, radius);
    if (ctol_d > radius)
        return 2;
    else if (ctol_d == radius)
        return 1;
    else
    {
        Point3f project_p = ptolProjection(c, l);
        if (isponl(project_p, l))
            return 1;
        else
            return 2;
    }
}

/*  判斷圓是否在多邊形內部
    polygon：多邊形的點
    c：要判斷的圓心
    radius：要判斷的半徑*/
bool Computational_geometry::isCircleInPolygon(const vector<Point3i>& polygon, const Point3i& c, double radius)
{
    if(isPointInPolygon(polygon, c))
    {
        Mat Contours=Mat::zeros(cv::Size(320,240),CV_8UC3);
        Contours.at<Vec3b>(Point(c.x, c.y))[0]=255;
        Contours.at<Vec3b>(Point(c.x, c.y))[1]=255;
        Contours.at<Vec3b>(Point(c.x, c.y))[2]=255;
        
        for (int i = 0; i < polygon.size(); ++i)
        {
            line(Contours, Point(polygon[i].x, polygon[i].y), Point(polygon[(i + 1) % polygon.size()].x, polygon[(i + 1) % polygon.size()].y), Scalar(89, 90, 90), 1);
            const Point3i& p1 = polygon[i];
            const Point3i& p2 = polygon[(i + 1) % polygon.size()];
            Line line(p1, p2, true);
            
            if (segToCircle(c, radius, line) != 2)
            {
                // imshow("inter_p",Contours);
                // waitKey(1);
                return false;
            }
        }
        circle(Contours, Point(c.x, c.y), radius, Scalar(0,255,0));
        // imshow("inter_p",Contours);
        // waitKey(1);
        return true;
    }
    else
    {
        return false;
    }
}