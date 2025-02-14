#include "utils.h"
#include <cmath>

const float kernel_1D[3] = {0.25, 0.5, 0.25};

uint8_t max(uint8_t a, uint8_t b)
{
    if(a<b)
    {
        return b;
    }
    return a;
}


uint8_t min(uint8_t a, uint8_t b)
{
    if(a<b)
    {
        return a;
    }
    return b;
}


float norm(float a, float b)
{
  return pow(pow(a,2)+pow(b,2), 0.5);
}

void gaussian_blur_horizontal(const uint8_t* input, uint8_t* temp, int width, int height) {
  for (int y = 0; y < height; y++) {
    for (int x = 1; x < width - 1; x++) {
      temp[y * width + x] = (
        input[y * width + (x - 1)] * kernel_1D[0] +
        input[y * width + x] * kernel_1D[1] +
        input[y * width + (x + 1)] * kernel_1D[2]
      );
    }
  }
}

// Step 2: Apply Gaussian blur vertically
void gaussian_blur_vertical(const uint8_t* temp, uint8_t* output, int width, int height) {
  for (int y = 1; y < height - 1; y++) {
    for (int x = 0; x < width; x++) {
      output[y * width + x] = (
        temp[(y - 1) * width + x] * kernel_1D[0] +
        temp[y * width + x] * kernel_1D[1] +
        temp[(y + 1) * width + x] * kernel_1D[2]
      );
    }
  }
}


void gaussian_blur(uint8_t *input, uint8_t *output, int width, int height) {
    const float kernel[3][3] = {
        {1, 2, 1},
        {2, 4, 2},
        {1, 2, 1}
    };
     uint8_t* temp = (uint8_t*)malloc(width * height);
     gaussian_blur_horizontal(input, temp, width, height);
     gaussian_blur_vertical(temp, output, width, height);
     free(temp);
}

void dilation_horizontal(const uint8_t* input, uint8_t* temp, int width, int height) {
  for (int y = 0; y < height; y++) {
    for (int x = 1; x < width - 1; x++) {
      temp[y * width + x] = (
        max(max(input[y * width + (x - 1)], input[y * width + x]), input[y * width + (x + 1)]));
    }
  }
}


void dilation_vertical(const uint8_t* temp, uint8_t* output, int width, int height) {
  for (int y = 1; y < height - 1; y++) {
    for (int x = 0; x < width; x++) {
      output[y * width + x] = max(max(temp[(y - 1) * width + x], temp[y * width + x]), temp[(y + 1) * width + x]);
    }
  }
}


void dilation(uint8_t *input, uint8_t *output, int width, int height) {
     uint8_t* temp = (uint8_t*)malloc(width * height);
     dilation_horizontal(input, temp, width, height);
     dilation_vertical(temp, output, width, height);
     free(temp);
}

void erosion_horizontal(const uint8_t* input, uint8_t* temp, int width, int height) {
  for (int y = 0; y < height; y++) {
    for (int x = 1; x < width - 1; x++) {
      temp[y * width + x] = (
        min(min(input[y * width + (x - 1)], input[y * width + x]), input[y * width + (x + 1)]));
    }
  }
}


void erosion_vertical(const uint8_t* temp, uint8_t* output, int width, int height) {
  for (int y = 1; y < height - 1; y++) {
    for (int x = 0; x < width; x++) {
      output[y * width + x] = max(max(temp[(y - 1) * width + x], temp[y * width + x]), temp[(y + 1) * width + x]);
    }
  }
}


void erosion(uint8_t *input, uint8_t *output, int width, int height) {

     uint8_t* temp = (uint8_t*)malloc(width * height);
     erosion_horizontal(input, temp, width, height);
     erosion_vertical(temp, output, width, height);
     free(temp);
}

void opening(uint8_t *input, uint8_t *output, int width, int height)
{
     uint8_t* temp = (uint8_t*)malloc(width * height);
     erosion(input, temp, width, height);
     dilation(temp, output, width, height);
     free(temp);
}