#ifndef SEG_H
#define SEG_H
#include <cstdint>
#include <vector>
#include <queue>
#include "hull.h"
#include "utils.h"

// Perform Segmentation on input to extract the object's convex hull. input is in dimensions width X height.
// The methodology is almost identical to the one in utils.py with some simplification for the esp.
std::vector<Hull> segment(uint8_t* input, int width, int height);



#endif