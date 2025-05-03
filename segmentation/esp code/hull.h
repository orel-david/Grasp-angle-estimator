#ifndef HULL_H
#define HULL_H
#include <vector>
#include <iostream>
#include <random>
#include <cmath>
#include <algorithm>


struct Point {
    // Struct for a 2D point.
    double x, y;

    Point(int x, int y) : x(x), y(y) {}

    Point(double x, double y) : x(x), y(y) {}
    ~Point() = default;


    // Order between the point is firstly by the x axis and then by the y axis.
    bool operator<(const Point& p) const {
        return (x == p.x) ? y < p.y : x < p.x;
    }

    // Create a point by subtracting 2 points.
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }
};

// Cross product of vectors(o->a) with (o->b)
int cross(const Point& o, const Point& a, const Point& b);

// Dot prduct of vectors a,b
double dot(const Point& a, const Point& b);

// Euclidean norm of point a from the origin (0,0)
double norm(const Point& a);

// Cosine similarity of vector a,b with origin (0,0)
double cosineSimilarity(const Point& a, const Point& b);

struct Circle
{
    // Struct for a circle by radius r and center.
    Point center;
    double r;
    ~Circle() = default;

};


// Returns whether p is inside circle c.
bool isInside(const Circle& c, const Point& p);

// This computes the circumcenter of a triangle formed by (0,0), (bx,by), and (cx,cy).
// Using the perpendicular bisector formula in 2D.
Point getCircleCenter(double bx, double by, double cx, double cy);

// Calculate bounding circle of points a,b,c
Circle circleFrom(const Point& a, const Point& b,   const Point& c);

// Calculate bounding circle of points a,b
Circle circleFrom(const Point& a, const Point& b);

// Helper for calculating the bounding circle method. Checks if a circle contains all the points.
bool isValidCircle(const Circle& c, const std::vector<Point>& points);

// Helper for calculating the bounding circle method. welzl helper function.
// Implementation inspired by http://geeksforgeeks.org/minimum-enclosing-circle-using-welzls-algorithm/
Circle minCircleHelper(std::vector<Point>& p, std::vector<Point> r, int n);

// Calculate min bounding circle using welzl algorithm 
Circle minCircle(const std::vector<Point>& p);

// Calculate min bounding rectangle with rotating calipers algorithm
std::pair<double, double> minBoundingRectangleAreaAndRatio(const std::vector<Point>& hull);

// Calculate the angle of point p from the optical axis when accounting for the focal length.
double angleFromOptical(const Point&p);

struct Hull
{
    // struct of convex hull.
    private:
    // The points on the convex hull.
    std::vector<Point> points;
    double center_x;
    double center_y;
    double area_rect; // area of min bounding rectangle
    double area_circ; // area of min bounding circle
    double area;
    float aspect_ratio;
    int valid; // lazy evaluation for validity
    // valid values: -1 unevaluated, 0 non valid, 1 ratio valid
    bool inCenter;  // bool to check whether a hull is in the center of the image

    public:
    Hull(std::vector<std::pair<int,int>> in_points);// Recieve a starting list op points and construct a hull object
    ~Hull() = default;
    void print() const;

    // calculate the convex hull of points set in.
    std::vector<Point> convexHull(std::vector<std::pair<int,int>>& in);

    // Calculate the hull area and center.
    void initAreaCenter();

    // Compare hulls by area
    bool operator<(const Hull& h) const;
    // Join hulls by connecting their points. 
    Hull operator+(const Hull& h) const;

    // Returns the poins of the convex hull.
    const std::vector<Point>& getPoints() const;

    Point getCenter() const; // Returns the hull center
    double getArea() const; // Returns the hull area

    double getRectArea(); // Returns the area of the min bounding rectangle
    float getAspectRatio();// Returns the aspect ratio of the min bounding rectangle.
    
    bool isInside(const Point& p) const; // Returns whether p is inside the convex hull.
};

#endif