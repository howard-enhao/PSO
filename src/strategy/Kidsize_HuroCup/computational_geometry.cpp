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

Point3i Computational_geometry::add(const Point3i& lhs, const Point3i& rhs)
{
    Point3i res;

    res.x = lhs.x + rhs.x;
    res.y = lhs.y + rhs.y;
    res.z = lhs.z + rhs.z;

    return res;
}

Point3i Computational_geometry::sub(const Point3i& lhs, const Point3i& rhs)
{
    Point3i res;

    res.x = lhs.x - rhs.x;
    res.y = lhs.y - rhs.y;
    res.z = lhs.z - rhs.z;

    return res;
}

Point3i Computational_geometry::mul(const Point3i& p, double ratio)
{
    Point3i res;

    res.x = p.x * ratio;
    res.y = p.y * ratio;
    res.z = p.z * ratio;

    return res;
}

Point3i Computational_geometry::div(const Point3i& p, double ratio)
{
    Point3i res;
    
    res.x = p.x / ratio;
    res.y = p.y / ratio;
    res.z = p.z / ratio;

    return res;
}

bool Computational_geometry::equal(const Point3i& lhs, const Point3i& rhs)
{
    return(lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

Point3i Computational_geometry::normalize(const Point3i& vec)
{
    Point3i res;

    res = div(vec, length(vec));

    return res;
}

double Computational_geometry::dotMultiply(const Point3i& vec1, const Point3i& vec2)
{
    return(vec1.x * vec2.x + vec1.y * vec2.y + 0 * 0/*vec1.z * vec2.z*/);
}

Point3i Computational_geometry::multiply(const Point3i& vec1, const Point3i& vec2)
{
    Point3i result;

    result.x = vec1.y * 0/*vec2.z*/ - vec2.y * 0/*vec1.z*/;
    result.y = 0/*vec1.z*/ * vec2.x - 0/*vec2.z*/ * vec1.x;
    result.z = vec1.x * vec2.y - vec2.x * vec1.y;

    return result;
}

double Computational_geometry::length(const Point3i& vec)
{
    return (sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2)));
}

double Computational_geometry::Cos(const Point3i& vec1, const Point3i& vec2)
{
    Point3i unit_vec1 = normalize(vec1);
    Point3i unit_vec2 = normalize(vec2);

    return dotMultiply(unit_vec1, unit_vec2);
}

bool Computational_geometry::isponl(const Point3i& p, const Line& l)
{
    Point3i line_vec = sub(l.e, l.s);
    Point3i point_vec1 = sub(p, l.s);
    Point3i point_vec2 = sub(p, l.e);

    Point3i mul_vec = multiply(line_vec, point_vec1);
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

bool Computational_geometry::isSegIntersect(const Line& l1, const Line& l2,Point3i& inter_p)
{
    Point3i line1 = sub(l1.e, l1.s);
    Point3i line2 = sub(l2.e, l2.s);
    Point3i norm1 = normalize(line1);
    Point3i norm2 = normalize(line2);
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
            double t1 = length(multiply(sub(l1.s, l2.s), norm2)) / length(multiply(norm2, norm1));
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

void Computational_geometry::boxOfPolygon(const vector<Point3i>& polygon, Point3i& down_left, Point3i& up_right)
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

    down_left = Point3i(min_x, min_y, 0);
    up_right = Point3i(max_x, max_y, 0);
}

bool Computational_geometry::isPointInPolygon(const vector<Point3i>& polygon, const Point3i& p)
{
    Point3i down_left, up_right;
    boxOfPolygon(polygon, down_left, up_right);
    cout<<down_left<<", "<<up_right<<endl;
    // 位於多邊形外部一點
    Point3i out_p = sub(down_left, Point3i(10.0, 10.0, 0.0));
    cout<<"out_p = "<<out_p<<endl;
    int cnt(0);
    Line p_line(p, out_p, true);
    for (int i = 0; i < polygon.size(); ++i)
    {
        Point3i s = polygon[i];
        Point3i e = polygon[(i + 1) % polygon.size()];
        Line seg(s, e, true);
        Point3i inter_p;
        if (isSegIntersect(p_line, seg, inter_p))
        {
            cnt++;
        }
    }

    return (cnt % 2 == 1);
}