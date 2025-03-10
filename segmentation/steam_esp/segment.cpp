#include "segment.h"
#include <cstring>
#include <algorithm>

double MIN_RATIO = 0.25;
double MAX_DIST = 30;

bool checkRatio(Hull h)
{
    // Return true if not valid
    float ratio = h.getAspectRatio();
    return ratio < MIN_RATIO || ratio > (1/MIN_RATIO);
}


bool hullInCenter(Hull h, int width, int height)
{
    int leftBound = width / 4;
    int rightBound = 3*width / 4;
    int x_center = h.getCenter().x;

    return leftBound <= x_center && x_center <= rightBound;
}


double approxDist(Hull a, Hull b)
{
    double minDist = UINT32_MAX;
    for(Point p1: a.getPoints())
    {
        for(Point p2: b.getPoints())
        {
            double dist = norm(p1-p2);
            if (dist < minDist)
            {
                minDist = dist;
            }
        }
    }

    return minDist;
}


std::vector<Hull> segment(uint8_t *input, int width, int height)
{
    uint8_t* output = (uint8_t*) calloc(width * height, sizeof(uint8_t));
    uint8_t* temp = (uint8_t*) calloc(width * height, sizeof(uint8_t));


    gaussian_blur(input, temp);
    gaussian_blur(temp, input);
    canny(input, output, 50, 150);

    std::vector<Hull> hulls = extract_hulls(output);
    free(output);
    free(temp);

    hulls.erase(std::remove_if(hulls.begin(), hulls.end(), checkRatio), hulls.end());
    if(hulls.size() == 0)
    {
      return {Hull({})};
    }

    bool merged = true;
    float largestArea = -1;
    Hull* largestHull = nullptr;

    for (size_t i = 0; i < 2; i++)
    {
        merged = false;
        for(Hull h: hulls)
        {
            if(h.getArea() > largestArea)
            {
                largestHull = &h;
                largestArea = h.getArea();
            }
        }

        if (largestArea == -1)
        {
            // No hull was found
            return {Hull({})};
        }
        std::vector<Hull> newHulls;
        Hull tmp_hull = *largestHull;
        for (Hull h : hulls)
        {
            if (!(&h == largestHull))
            {
                double dist = approxDist(tmp_hull, h);
                if(dist < MAX_DIST && hullInCenter(h, width, height))
                {
                    // merged = true;
                    tmp_hull = tmp_hull + h;
                }else
                {
                    newHulls.push_back(h);
                } 
            }
        }

        if(!merged)
        {
            return {tmp_hull};
        }
        newHulls.push_back(tmp_hull);
        hulls = newHulls;
        largestArea = tmp_hull.getArea();
        largestHull = &tmp_hull; // problem
    }
    return {*largestHull};
}

