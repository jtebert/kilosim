#ifndef SHAPES_H
#define SHAPES_H

#include <vector>
#include <string>

typedef struct point_t {
    double x;
    double y;
} point_t;

typedef struct polygon_c_t {
    std::vector<point_t> points;
    std::vector<float> color;
} polygon_c_t;

typedef struct rect_c_t {
    point_t pos;  // lower right corner
    float width;
    float height;
    std::vector<float> color;
} rect_c_t;

typedef struct circle_t {
    // x, y define center of circle
    double x;
    double y;
    double rad;
} circle_t;

std::vector<polygon_c_t> gen_color_polys(std::string filename);
std::vector<rect_c_t> gen_color_rects(std::string filename);
std::string float_to_string(float num);

bool point_in_polygon(point_t point, polygon_c_t polygon);
bool point_in_rect(point_t point, rect_c_t rect);
bool point_in_circle(point_t point, circle_t circ);

#endif