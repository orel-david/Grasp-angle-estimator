#include "segment.h"

Hull segment(uint8_t *input, int width, int height)
{
    uint8_t* output = (uint8_t*) calloc(width * height, sizeof(uint8_t));

    gaussian_blur(input, output);
    gaussian_blur(output, input);
    canny(input, output, 50, 150);
    std::vector<Hull> hulls = extract_hulls(output);

    
}
