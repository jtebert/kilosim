#ifndef SHAPES_H
#define SHAPES_H

#include <sys/stat.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

typedef struct point_t {
    double x;
    double y;
} point_t;

typedef struct polygon_t {
    std::vector<point_t> points;
    std::vector<float> color;
} polygon_t;

typedef struct circle_t {
    // x, y define center of circle
    double x;
    double y;
    double rad;
} circle_t;

std::vector<polygon_t> gen_color_squares(std::string filename);
std::string float_to_string(float num);




#endif