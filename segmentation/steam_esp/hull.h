#ifndef HULL_H
#define HULL_H
#include <vector>
#include <iostream>
#include <cmath>

struct Hull
{
    std::vector<std::pair<int,int>> points;
    int center_x;
    int center_y;
    int area_rect;
    int area_circ;
    int area;
    float aspect_ratio;

    Hull(std::vector<std::pair<int,int>> in_points): points(in_points), center_x(-1), 
        center_y(-1), area_rect(-1), area_circ(-1), area(-1), aspect_ratio(-1)
    {// TODO: replace the initialization of points to be the convex hull
    }

    void print() const {
        std::cout << "Hull " << ": " << std::endl;
        for (size_t i = 0; i < points.size(); ++i) {
            std::cout << "(" << points[i].first << ", " << points[i].second << ") ";
            std::cout << std::endl;
        }
    }


    std::vector<std::pair<int,int>> convexHull(std::vector<std::pair<int,int>> in)
    {
        //TODO
    }


    void initAreaCenter()
    {
        //TODO
    }
};

#endif