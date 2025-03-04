#include "utils.h"
#include <cmath>
#include <Arduino.h>

#define HEIGHT 240
#define WIDTH 320

const float kernel_1D[3] = {0.25, 0.5, 0.25};


const int SOBEL_X[3][3] = {
  {-1,  0,  1},
  {-2,  0,  2},
  {-1,  0,  1}
};


const int SOBEL_Y[3][3] = {
  {-1, -2, -1},
  { 0,  0,  0},
  { 1,  2,  1}
};

std::queue<std::pair<int, int>> candidates;

uint8_t* imageFromHulls(const std::vector<Hull>& hulls) {
  uint8_t* output = (uint8_t*) calloc(WIDTH * HEIGHT, sizeof(uint8_t));
  if (!output) return nullptr; 

  for (const Hull& h : hulls) { 
      for (const Point& p : h.getPoints()) { 
          output[int(p.y) * WIDTH + int(p.x)] = 255; 
      }
  }

  return output;
}

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


void gaussian_blur_horizontal(const uint8_t* input, uint8_t* temp) {
  for (int y = 0; y < HEIGHT; y++) {
    for (int x = 1; x < WIDTH - 1; x++) {
      temp[y * WIDTH + x] = (
        input[y * WIDTH + (x - 1)] * kernel_1D[0] +
        input[y * WIDTH + x] * kernel_1D[1] +
        input[y * WIDTH + (x + 1)] * kernel_1D[2]
      );
    }
  }
}


void gaussian_blur_vertical(const uint8_t* temp, uint8_t* output) {
  for (int y = 1; y < HEIGHT - 1; y++) {
    for (int x = 0; x < WIDTH; x++) {
      output[y * WIDTH + x] = (
        temp[(y - 1) * WIDTH + x] * kernel_1D[0] +
        temp[y * WIDTH + x] * kernel_1D[1] +
        temp[(y + 1) * WIDTH + x] * kernel_1D[2]
      );
    }
  }
}


void gaussian_blur(uint8_t *input, uint8_t *output) {
    const float kernel[3][3] = {
        {1, 2, 1},
        {2, 4, 2},
        {1, 2, 1}
    };
     uint8_t* temp = (uint8_t*)malloc(WIDTH * HEIGHT);
     gaussian_blur_horizontal(input, temp);
     gaussian_blur_vertical(temp, output);
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


void sobelFilter(uint8_t *input, uint8_t* grad, uint8_t* angle){
  for (int y = 3; y < HEIGHT - 3; y++) {
    for (int x = 3; x < WIDTH - 3; x++) {
      int gx = (-input[(y-1)*WIDTH + (x-1)] - 2*input[y*WIDTH + (x-1)] - input[(y+1)*WIDTH + (x-1)]) +
                    ( input[(y-1)*WIDTH + (x+1)] + 2*input[y*WIDTH + (x+1)] + input[(y+1)*WIDTH + (x+1)]);

      int gy = (-input[(y-1)*WIDTH + (x-1)] - 2*input[(y-1)*WIDTH + x] - input[(y-1)*WIDTH + (x+1)]) +
                    ( input[(y+1)*WIDTH + (x-1)] + 2*input[(y+1)*WIDTH + x] + input[(y+1)*WIDTH + (x+1)]);

      grad[y * WIDTH + x] = min(255, abs(gx) + abs(gy));

      // Approximate angle instead of atan2
      uint8_t a = (gy > 0) ? (gx > 0 ? 45 : 135) : (gx > 0 ? 315 : 225);
      angle[y * WIDTH + x] = a;
      }
  }
}


void nonMaximumSuppression(uint8_t* grad, uint8_t* direction, uint8_t* output) {
  for (int y = 1; y < HEIGHT - 1; y++) {
      for (int x = 1; x < WIDTH - 1; x++) {
          uint8_t angle = direction[y*WIDTH + x];

          uint8_t q = 255, r = 255;
          if ((0 <= angle < 22.5) || (157.5 <= angle <= 180)) {
              q = grad[y*WIDTH + x+1];
              r = grad[y*WIDTH + x-1];
          } else if (22.5 <= angle < 67.5) {
              q = grad[(y-1)*WIDTH + x+1];
              r = grad[(y+1)*WIDTH + x-1];
          } else if (67.5 <= angle < 112.5) {
            q = grad[(y-1)*WIDTH + x];
            r = grad[(y+1)*WIDTH + x];
          } else if (112.5 <= angle < 157.5) {
            q = grad[(y-1)*WIDTH + x-1];
            r = grad[(y+1)*WIDTH + x+1];
          }

          if (grad[y*WIDTH + x] >= q && grad[y*WIDTH + x] >= r)
              output[y*WIDTH + x] = grad[y*WIDTH + x];
          else
              output[y*WIDTH + x] = 0;
      }
  }
}


void doubleThreshold(uint8_t* input, uint8_t* output, int low, int high){
  for (int y = 1; y < HEIGHT - 1; y++) {
    for (int x = 1; x < WIDTH - 1; x++) {
      if (input[y*WIDTH + x] >= high){
        candidates.push({x,y});
        output[y*WIDTH + x] = 255;
    }else if (input[y*WIDTH + x] >= low){
        output[y*WIDTH + x] = 128;
    }
      else{
        output[y*WIDTH + x] = 0;
      }
    }
  }

  // Hysteresis: Convert weak edges (128) to strong if connected to 255
  for (int y = 1; y < HEIGHT - 1; y++) {
    for (int x = 1; x < WIDTH - 1; x++) {
      if (output[y*WIDTH + x] == 128) {
        if (output[(y-1)*WIDTH + x-1] == 255 || output[(y-1)*WIDTH + x] == 255 || output[(y-1)*WIDTH + x+1] == 255 ||
            output[y*WIDTH + x-1] == 255 || output[y*WIDTH + x+1] == 255 ||
            output[(y+1)*WIDTH + x-1] == 255 || output[(y+1)*WIDTH + x] == 255 || output[(y+1)*WIDTH + x+1] == 255)
       {
          output[y*WIDTH + x] = 255;
          candidates.push({x,y});
        } else {
          output[y*WIDTH + x] = 0;
        }
      }
    }
  }
}

void canny(uint8_t *input, uint8_t *output, int low, int high)
{
  // We assume blurred input
  uint8_t* grad = (uint8_t*) calloc(WIDTH * HEIGHT, sizeof(uint8_t));
  uint8_t* angle = (uint8_t*) malloc(WIDTH * HEIGHT);
  uint8_t* temp = (uint8_t*) calloc(WIDTH * HEIGHT, sizeof(uint8_t));
  if (!grad || !angle || !input || !temp) {
    // TODO Handle it better
    return;
}

  sobelFilter(input, grad, angle);
  // nonMaximumSuppression(grad, angle, temp);
  doubleThreshold(grad, output, low, high);
  free(temp);
  free(grad);
  free(angle);

}


int dirX[4] = {1, 0, -1, 0};
int dirY[4] = {0, 1, 0, -1};

bool isValid(int x, int y, uint8_t* image, std::vector<std::vector<bool>>& visited) {
    return x >= 5 && x < WIDTH -5 && y >= 5 && y < HEIGHT-5 && image[y*WIDTH + x] == 255 && !visited[y][x];
}

// Simple flood fill to find contours
void findContour(int startX, int startY, uint8_t* image, std::vector<std::vector<bool>>& visited, std::vector<std::pair<int, int>>& contour) {
    std::queue<std::pair<int, int>> q;
    q.push({startX, startY});
    visited[startY][startX] = true;

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();
        contour.push_back({x, y});

        // Check all 4 directions
        for (int i = 0; i < 4; ++i) {
            int newX = x + dirX[i];
            int newY = y + dirY[i];

            if (isValid(newX, newY, image, visited)) {
                visited[newY][newX] = true;
                q.push({newX, newY});
            }
        }
    }
}



// require fixing
std::vector<Hull> extract_hulls(uint8_t* image)
{
  std::vector<std::vector<bool>> visited(HEIGHT, std::vector<bool>(WIDTH, false));
  std::vector<Hull> hulls;

  while(!candidates.empty())
  {
      auto [x, y] = candidates.front();
      candidates.pop();
      if (!visited[y][x]){
      std::vector<std::pair<int, int>> contour;
      findContour(x, y, image, visited, contour);
      Hull hull = Hull(contour);
      hulls.push_back(hull);
      }
  }

  return hulls;
}