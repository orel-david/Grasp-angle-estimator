#ifndef HULL_H
#define HULL_H
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

struct Point {
    int x, y;

    Point(int x, int y) : x(x), y(y) {}


    bool operator<(const Point& p) const {
        return (x == p.x) ? y < p.y : x < p.x;
    }
};

int cross(const Point& o, const Point& a, const Point& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

struct Hull
{
    std::vector<Point> points;
    int center_x;
    int center_y;
    int area_rect;
    int area_circ;
    int area;
    float aspect_ratio;

    Hull(std::vector<std::pair<int,int>> in_points): center_x(-1), 
        center_y(-1), area_rect(-1), area_circ(-1), area(-1), aspect_ratio(-1)
    {
        this->points = this->convexHull(in_points);
        this->initAreaCenter();
        this->print();
    }

    void print() const {
        std::cout << "Hull " << ": " << std::endl;
        for (size_t i = 0; i < points.size(); ++i) {
            std::cout << "(" << points[i].x << ", " << points[i].y << ") ";
            std::cout << std::endl;
        }
    }


    std::vector<Point> convexHull(std::vector<std::pair<int,int>>& in)
    {
        std::vector<Point> points;
        points.reserve(in.size());

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


    void initAreaCenter()
    {
        //TODO
    }
};

#endif