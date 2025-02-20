#ifndef UTILS_H
#define UTILS_H
#include <cstdint>
#include <iostream>
#include <vector>
#include <queue>

float norm(float a, float b);

void gaussian_blur(uint8_t *input, uint8_t *output);
void opening(uint8_t *input, uint8_t *output, int height, int width);
void canny(uint8_t *input, uint8_t *output, int low, int high);
std::vector<std::vector<std::pair<int, int>>> extract_contours(uint8_t* image);

uint8_t max(uint8_t a, uint8_t b);
uint8_t min(uint8_t a, uint8_t b);

#endif