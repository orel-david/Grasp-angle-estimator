#ifndef UTILS_H
#define UTILS_H
#include <cstdint>
#include <iostream>
#include <vector>
#include <queue>
#include "hull.h"


float norm(float a, float b); // Calculate euclidean norm of vector (a,b)

void gaussian_blur(uint8_t *input, uint8_t *output); // Applies gaussian blur to the input image. Saves in output array.
void opening(uint8_t *input, uint8_t *output, int height, int width); // Applies opening transformation on input and saves in output. 
void canny(uint8_t *input, uint8_t *output, int low, int high); // Perform canny edge detection on input and saves in output. low and high are the edges thresholds.
std::vector<Hull> extract_hulls(uint8_t* image); // extract the contours from a binary image. Returns an array of Hull objects from hull.h.

uint8_t* imageFromHulls(const std::vector<Hull>& hulls); // Draws hulls on a 2D image in dimension WIDTHxHEIGHT(defined in utils.cpp).
uint8_t max(uint8_t a, uint8_t b); // Max(a,b)
uint8_t min(uint8_t a, uint8_t b); // Min(a, b)

#endif