#ifndef HULL_H
#define HULL_H
#include <vector>
#include <iostream>
#include <random>
#include <cmath>
#include <algorithm>


struct Point {
    double x, y;

    Point(int x, int y) : x(x), y(y) {}

    Point(double x, double y) : x(x), y(y) {}
    ~Point() = default;



    bool operator<(const Point& p) const {
        return (x == p.x) ? y < p.y : x < p.x;
    }

    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }
};


int cross(const Point& o, const Point& a, const Point& b);


double dot(const Point& a, const Point& b);


double norm(const Point& a);


double cosineSimilarity(const Point& a, const Point& b);

struct Circle
{
    Point center;
    double r;
    ~Circle() = default;

};


bool isInside(const Circle& c, const Point& p);


Point getCircleCenter(double bx, double by, double cx, double cy);


Circle circleFrom(const Point& a, const Point& b,   const Point& c);


Circle circleFrom(const Point& a, const Point& b);


bool isValidCircle(const Circle& c, const std::vector<Point>& points);


Circle minCircleHelper(std::vector<Point>& p, std::vector<Point> r, int n);


Circle minCircle(const std::vector<Point>& p);


std::pair<double, double> minBoundingRectangleAreaAndRatio(const std::vector<Point>& hull);

struct Hull
{
    private:
    std::vector<Point> points;
    double center_x;
    double center_y;
    double area_rect;
    double area_circ;
    double area;
    float aspect_ratio;
    int valid; // lazy evaluation for validity
    // valid values: -1 unevaluated, 0 non valid, 1 ratio valid
    bool inCenter; 

    public:
    Hull(std::vector<std::pair<int,int>> in_points);
    ~Hull() = default;
    void print() const;


    std::vector<Point> convexHull(std::vector<std::pair<int,int>>& in);


    void initAreaCenter();

    bool operator<(const Hull& h) const;
    Hull operator+(const Hull& h) const;


    const std::vector<Point>& getPoints() const;

    Point getCenter() const;
    double getArea() const;

    double getRectArea();
    float getAspectRatio();
    
    bool isInside(const Point& p) const;
};

#endif