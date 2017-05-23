#ifndef SHAPES_H
#define SHAPES_H

#include <sys/stat.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#ifndef STAT_INFO
#define STAT_INFO
struct stat info;
#endif

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

//std::vector<polygon_t> gen_color_squares(std::string filename);
//std::string float_to_string(float num);

std::vector<polygon_t> gen_color_squares(std::string filename) {
    // Generate the list of polygons (colored squares) included in the given file

    if (stat(filename.c_str(), &info) != 0) {
        printf("Error: Shapes file '%s' does not exist.", filename.c_str());
        exit(1);
    }

    std::vector<polygon_t> polys;
    std::ifstream infile(filename);
    float x1, y1, x2, y2, x3, y3, x4, y4, c1, c2, c3;
    std::string line;

    while (infile >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> c1 >> c2 >> c3) {
        polygon_t new_poly = {{{x1,y1}, {x2,y2}, {x3,y3}, {x4,y4}}, {c1, c2, c3}};
        polys.push_back(new_poly);
    }
    return polys;
}

std::string float_to_string(float num) {
    std::string str = std::to_string(num);
    str.erase(str.find_last_not_of('0') + 1, std::string::npos);
    return str;
}


#endif