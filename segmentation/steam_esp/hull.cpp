#include "hull.h"

#define FOCAL_LENGTH 292 // in pixels, it is 3.21mm and pixel is 2.2 microns, adjusted to resolution

int sign(double x)
{
  if(x < 0)
    return -1;
  return 1;
}

int cross(const Point& o, const Point& a, const Point& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}


double dot(const Point& a, const Point& b) {
    return a.x * b.x + a.y * b.y;
}


double norm(const Point& a) {
    return std::sqrt(a.x * a.x + a.y * a.y);
}


double cosineSimilarity(const Point& a, const Point& b) {
    double norm_a = norm(a);
    double norm_b = norm(b);

    if (norm_a == 0 || norm_b == 0) {
        return 0;
    }

    return dot(a, b) / (norm_a * norm_b);
}

bool isInside(const Circle& c, const Point& p) {
    return norm(c.center - p) <= c.r;
}


Point getCircleCenter(double bx, double by, double cx, double cy) {
    double b = bx * bx + by * by;
    double c = cx * cx + cy * cy;
    double d = bx * cy - by * cx;
    return { (cy * b - by * c) / (2 * d), (bx * c - cx * b) / (2 * d) };
}


Circle circleFrom(const Point& a, const Point& b,   const Point& c) {
    Point i = getCircleCenter(b.x - a.x, b.y - a.y, c.x - a.x, c.y - a.y);

    i.x += a.x;
    i.y += a.y;
    return { i, norm(i - a) };
}


Circle circleFrom(const Point& a, const Point& b) {

    // Set the center to be the midpoint of a and b
    Point c = { (a.x + b.x) / 2.0, (a.y + b.y) / 2.0 };

    return {c, norm(a - b) / 2.0 };
}


bool isValidCircle(const Circle& c, const std::vector<Point>& points) {
    for (const Point& p : points){
        if (!isInside(c, p)){
            return false;
        }
    }
    return true;
}


Circle minCircleHelper(std::vector<Point>& p, std::vector<Point> r, int n) {

    // Base case when all points processed or |r| = 3
    if (n == 0 || r.size() == 3) {
        if (r.empty()) {
            return { { 0, 0 }, 0 };
        }
        else if (r.size() == 1) {
            return {r[0], 0};
        }
        else if (r.size() == 2) {
            return circleFrom(r[0], r[1]);
        }
    
        // check for a pair that contains the third point
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
    
                Circle c = circleFrom(r[i], r[j]);
                if (isValidCircle(c, r))
                    return c;
            }
        }
        return circleFrom(r[0], r[1], r[2]);
    }

    // Pick a random point randomly
    int idx = rand() % n;
    Point pnt = p[idx];

    // Put the picked point at the end of p
    // since it's more efficient than
    // deleting from the middle of the vector
    std::swap(p[idx], p[n - 1]);

    // Get the MEC circle d from the
    // set of points p - {p}
    Circle d = minCircleHelper(p, r, n - 1);

    // If d contains pnt, return d
    if (isInside(d, pnt)) {
        return d;
    }

    // Otherwise, must be on the boundary of the MEC
    r.push_back(pnt);

    // Return the MEC for p - {p} and r U {p}
    return minCircleHelper(p, r, n - 1);
}


Circle minCircle(const std::vector<Point>& p) {
    std::vector<Point> pCopy = p;
    std::random_device rd;
    std::mt19937 g(rd()); 

    std::shuffle(pCopy.begin(), pCopy.end(), g);
    return minCircleHelper(pCopy, {}, pCopy.size());
}


std::pair<double, double> minBoundingRectangleAreaAndRatio(const std::vector<Point>& hull) {
    int n = hull.size();
    if (n < 3) return {0,0}; // No valid rectangle


    double minArea = 1e20;
    int w,h =0;

    for (int i = 0; i < n; i++) {
        int base = (i+1) %n;
        Point p1 = hull[i];
        Point p2 = hull[base];
        Point u0 = Point(p2.x - p1.x, p2.y-p1.y);
        double uNorm = norm(u0);
        u0.x = u0.x/uNorm;
        u0.y = u0.y/uNorm;
        Point u1 = Point(-u0.y, u0.x);
        Point support[4] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}; 
        
        for(int j = 0; j < n; j++)
        {
            Point diff = hull[j] - p2;
            Point v = {dot(u0, diff), dot(u1, diff)};

            if(v.x > support[1].x || (v.x == support[1].x && v.y > support[1].y))
            {
                support[1] = v;
            }
            
            if (v.y > support[2].y || (v.y == support[2].y && v.x < support[2].x))
            {
                support[2] = v;
            }

            if(v.x < support[3].x || (v.x == support[3].x && v.y < support[3].y))
            {
                support[3] = v;
            }
        }
        double area = (support[1].x - support[3].x) * (support[2].y - support[0].y);

        if (area < minArea)
        {
            minArea = area;
            w = (support[1].x - support[3].x);
            h = (support[2].y - support[0].y);
        }
    }

    double ratio;
    if(w == 0 || h == 0)
    {
        ratio = 0;
    }
    else
    {
        ratio = w / h;
    }

    return {minArea, ratio};
}

double angleFromOptical(const Point &p)
{
    // Return angle in radians.
    
    int dx = p.x - 160; //width is 320
    int dy = p.y - 120; //height is 120

    double dist = std::sqrt(dx*dx + dy*dy);

    return std::atan(dist / FOCAL_LENGTH) * sign(dx);
}

Hull::Hull(std::vector<std::pair<int,int>> in_points): center_x(-1), 
        center_y(-1), area_rect(-1), area_circ(-1), area(-1), aspect_ratio(-1)
{
    this->points = this->convexHull(in_points);
    this->initAreaCenter();
}

void Hull::print() const {
    std::cout << "Hull" << ": " << std::endl;
    for (size_t i = 0; i < points.size(); ++i) {
        std::cout << "(" << points[i].x << ", " << points[i].y << ") ";
        std::cout << std::endl;
    }
    std::cout<< "Area: " << this->area<< " center: (" << this->center_x<< ", "<< this->center_y<< ")" << std::endl;
}


std::vector<Point> Hull::convexHull(std::vector<std::pair<int,int>>& in)
{
    std::vector<Point> points;
    points.reserve(in.size());
    if(in.size() == 0)
    {
      return points;
    }

    for (const auto& p : in) {
        points.emplace_back(p.first, p.second);
    }

    std::sort(points.begin(), points.end());
    std::vector<Point> lower, upper;
    
    for (const auto& p : points) {
        while (lower.size() >= 2 && cross(lower[lower.size() - 2], lower.back(), p) <= 0)
            lower.pop_back();
        lower.push_back(p);
    }
    
    for (int i = points.size() - 1; i >= 0; i--) {
        while (upper.size() >= 2 && cross(upper[upper.size() - 2], upper.back(), points[i]) <= 0)
            upper.pop_back();
        upper.push_back(points[i]);
    }
    
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());

    return lower;
}


void Hull::initAreaCenter()
{
    double A = 0;
    double CX = 0;
    double CY = 0;
    int n = this->points.size();
    if(n==0)
    {
      this->area = 0;
      this->center_x = 0;
      this->center_y = 0;
      return;
    }
    for (int i = 0; i < n; i++)
    {
        Point p1 = this->points[i];
        Point p2 = this->points[(i+1) % (n)];
        double det = p1.x * p2.y - p2.x * p1.y;
        A += det;
        CX += det * (p1.x + p2.x);
        CY += det * (p1.y + p2.y);
    }
    A = 0.5 * std::abs(A);
    CX = CX / (6*A);
    CY = CY / (6*A);
    this->area = A;
    this->center_x = CX;
    this->center_y = CY;
}

bool Hull::operator<(const Hull& h) const {
    return this->area < h.getArea();
}

Hull Hull::operator+(const Hull &h) const
{
    std::vector<std::pair<int,int>> points;
    std::vector<Point> in_p = h.getPoints();
    points.reserve(this->points.size() + in_p.size());
    for (const auto& p: this->points)
    {
        points.emplace_back(p.x, p.y);
    }

    for (const auto& p: in_p)
    {
        points.emplace_back(p.x, p.y);
    }
    Hull result = Hull(points);
    return result;
}

const std::vector<Point>& Hull::getPoints() const
{
    return this->points;
}

Point Hull::getCenter() const
{
    return {this->center_x, this->center_y};
}

double Hull::getArea() const
{
    return this->area;
}

double Hull::getRectArea() 
{
    // Gets the min bounding rect area
    if(this->area_rect == -1)
    {
        std::pair<double, double> temp = minBoundingRectangleAreaAndRatio(this->getPoints());
        this->area_rect = temp.first;
        this->aspect_ratio = temp.second;
        return temp.first;
    }

    return this->area_rect;
}


float Hull::getAspectRatio() 
{
    // Gets the min bounding rect area
    if(this->area_rect == -1)
    {
        std::pair<double, double> temp = minBoundingRectangleAreaAndRatio(this->getPoints());
        this->area_rect = temp.first;
        this->aspect_ratio = temp.second;
        return temp.second;
    }

    return this->aspect_ratio;
}


bool Hull::isInside(const Point& p) const
{
    int n = this->points.size();
    Point C = this->getCenter();
    for(int i=0; i < n; i++)
    {
        int next = (i+1) % n;
        Point A = this->points[i];
        Point B = this->points[next];
        double detA = cross(A, B ,p);
        double detB = cross(B, C, p);
        double detC = cross(C, A, p);

        bool hasNeg = false;
        bool hasPos = false;

        if(detA > 0 || detB > 0 || detC > 0)
        {
            hasPos = true;
        }

        if(detA < 0 || detB < 0 || detC < 0)
        {
            hasNeg = true;
        }

        if (!(hasNeg & hasPos))
        {
            return true;
        }
    }

    return false;
}
