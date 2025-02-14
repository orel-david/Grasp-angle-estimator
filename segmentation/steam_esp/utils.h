#ifndef UTILS_H
#define UTILS_H
#include <cstdint>
float norm(float a, float b);

void gaussian_blur(uint8_t *input, uint8_t *output, int width, int height);
void opening(uint8_t *input, uint8_t *output, int width, int height);
void erosion(uint8_t *input, uint8_t *output, int width, int height);
void dilation(uint8_t *input, uint8_t *output, int width, int height);
uint8_t max(uint8_t a, uint8_t b);
uint8_t min(uint8_t a, uint8_t b);

#endif