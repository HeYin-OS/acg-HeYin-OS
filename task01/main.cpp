#include <cstdio>
#include <iostream>
#include <cassert>
#include <vector>
#include <filesystem>
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/**
 * @brief compute the area of a triangle
 * @return area
 */
float area_of_a_triangle(
    float x0, float y0,
    float x1, float y1,
    float x2, float y2) {
  return ((x1 - x2) * (y0 - y2) - (x0 - x2) * (y1 - y2)) * 0.5f;
}

/**
 * @brief draw a triangle using the barycentric coordinates
 */
void draw_triangle(
    float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    std::vector<unsigned char> &img_data, unsigned int width, unsigned int height,
    unsigned char brightness) {
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto x = (float) iw + 0.5f;
      const auto y = (float) ih + 0.5f;
      const auto a01 = area_of_a_triangle(x, y, x0, y0, x1, y1);
      const auto a12 = area_of_a_triangle(x, y, x1, y1, x2, y2);
      const auto a20 = area_of_a_triangle(x, y, x2, y2, x0, y0);
      if (a01 > 0.f && a12 > 0.f && a20 > 0.f) {
        img_data[ih * height + iw] = brightness;
      }
    }
  }
}

/**
 * @brief draw a triangle using the barycentric coordinates
 * @param polygon_xy xy coordinates of the corners of the polygon (counter clockwise order)
 * @param brightness brightness of the painted pixel
 */
void draw_polygon(
    const std::vector<float> &polygon_xy,
    std::vector<unsigned char> &img_data, unsigned int width, unsigned int height,
    unsigned int brightness) {
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto x = float(iw) + 0.5f; // x-coordinate of the center of the pixel
      const auto y = float(ih) + 0.5f; // y-coordinate of the center of the pixel
      const unsigned int num_vtx = polygon_xy.size() / 2;
      float winding_number = 0.0;
      for (unsigned int iedge = 0; iedge < num_vtx; ++iedge) {
        unsigned int i0_vtx = iedge;
        unsigned int i1_vtx = (iedge + 1) % num_vtx;
        // positions of the end points of the edge relative to (x,y)
        float p0x = polygon_xy[i0_vtx * 2 + 0] - x;
        float p0y = polygon_xy[i0_vtx * 2 + 1] - y;
        float p1x = polygon_xy[i1_vtx * 2 + 0] - x;
        float p1y = polygon_xy[i1_vtx * 2 + 1] - y;
        // write a few lines of code to compute winding number (hint: use atan2)
        static const double pi = 3.14159265358979323846264338327950288;
        double magnitude_product = sqrt(pow(p0x,2.0) + pow(p0y,2.0)) * sqrt(pow(p1x,2.0) + pow(p1y,2.0)); //magnitude product of p0 and p1
        double cosine_value = (p0x * p1x + p0y * p1y) / magnitude_product; //calculate the cosine value of the angle
        double sine_value = (p0x * p1y - p0y * p1x) / magnitude_product; //calculate the sine value of the angle
        winding_number += static_cast<float>(-atan2(sine_value,cosine_value) / (2 * pi)); //added into sum of winding number
      }
      const int int_winding_number = int(std::round(winding_number));
      if (int_winding_number == 1 ) { // if (x,y) is inside the polygon
        img_data[ih*width + iw] = brightness;
      }
    }
  }
}

/**
 * @brief draw a line using DDA algorithm
 * @param x0 x-coordinate of the first end point
 * @param y0 y-coordinate of the first end point
 * @param x1 x-coordinate of the second end point
 * @param y1 y-coordinate of the second end point
 * @param brightness brightness of the painted pixel
 */
void dda_line(
    float x0, float y0,
    float x1, float y1,
    std::vector<unsigned char> &img_data,
    unsigned int width,
    unsigned char brightness ) {
  auto dx = x1 - x0;
  auto dy = y1 - y0;
  // write some code below to paint pixel on the line with color `brightness`
  float slope;
  int end_x = static_cast<int>(x1);
  int end_y = static_cast<int>(y1);
  if (dx == 0){//strictly vertical line
      int pix_x = static_cast<int>(x0);
      int pix_y = static_cast<int>(y0);
      if(y1 > y0){
          do {
              img_data[width * pix_y + pix_x] = brightness;
              ++pix_y;
          } while (pix_y <= end_y);
      } else {
          do {
              img_data[width * pix_y + pix_x] = brightness;
              --pix_y;
          } while (pix_y >= end_y);
      }
      return;
  } else {
      slope = dy / dx; //slope of the line
  }
  //normal circumstances
  if (abs(slope) <= 1.0) { //choose the direction
      int ix = static_cast<int>(x0), i=0;
      if(x1 > x0){
          do {
              int pix_x = ix + i, pix_y = static_cast<int>(y0 + static_cast<float>(i) * slope);
              img_data[width * pix_y + pix_x] = brightness;
              ++i;
          } while (ix + i <= end_x);
      } else {
          do {
              int pix_x = ix - i, pix_y = static_cast<int>(y0 - static_cast<float>(i) * slope);
              img_data[width * pix_y + pix_x] = brightness;
              ++i;
          } while (ix - i >= end_x);
      }
  } else {
      int iy = static_cast<int>(y0), i=0;
      if(y1 > y0){
          do {
              int pix_y = iy + i, pix_x = static_cast<int>(x0 + static_cast<float>(i) / slope);
              img_data[width * pix_y + pix_x] = brightness;
              ++i;
          } while (iy + i <= end_y);
      } else {
          do {
              int pix_y = iy - i, pix_x = static_cast<int>(x0 - static_cast<float>(i) / slope);
              img_data[width * pix_y + pix_x] = brightness;
              ++i;
          } while (iy - i >= end_y);
      }
  }
}

int main() {
  constexpr unsigned int width = 100;
  constexpr unsigned int height = 100;
  std::vector<unsigned char> img_data(width * height, 255); // white initial image
  draw_triangle(5., 5., 15., 45., 45., 15., img_data, width, height, 128);
  draw_polygon(
      {55., 5.,75., 75.,15., 55.,15., 95.,95., 95.,95., 5.}, // the pix coordinates of the corners of the polygon
      img_data, width, height, // image data, size
      200); // brightness to draw
  dda_line(50.0, 50.0, 72.8, 10.0, img_data, width, 0); // right-down
  dda_line(50.0, 50.0, 90.0, 27.2, img_data, width, 0); // right-down
  dda_line(50.0, 50.0, 72.8, 90.0, img_data, width, 0); // right-up
  dda_line(50.0, 50.0, 90.0, 72.8, img_data, width, 0); // right-up
  dda_line(50.0, 50.0, 10.0, 72.8, img_data, width, 0); // left-up
  dda_line(50.0, 50.0, 27.2, 90.0, img_data, width, 0); // left-up
  dda_line(50.0, 50.0, 10.0, 27.2, img_data, width, 0); // left-down
  dda_line(50.0, 50.0, 27.2, 10.0, img_data, width, 0); // left-down
  dda_line(50.0, 50.0, 90.0, 50.0, img_data, width, 0); // right
  dda_line(50.0, 50.0, 50.0, 90.0, img_data, width, 0); // up
  dda_line(50.0, 50.0, 10.0, 50.0, img_data, width, 0); // left
  dda_line(50.0, 50.0, 50.0, 10.0, img_data, width, 0); // down
  // save gray scale image with 1 byte depth
  stbi_write_png(
      (std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png").string().c_str(),
      width, height, 1,img_data.data(), width);
}
