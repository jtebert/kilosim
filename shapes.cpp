//
// Created by jtebert on 5/24/17.
//

#include "shapes.h"
#include <sys/stat.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

std::vector<polygon_c_t> gen_color_polys(std::string filename) {
    // Generate the list of polygons (colored squares) included in the given file
    struct stat file_info;
    if (stat(filename.c_str(), &file_info) != 0) {
        printf("Error: Shapes file '%s' does not exist.\n", filename.c_str());
        exit(1);
    }

    std::vector<polygon_c_t> polys;
    std::ifstream infile(filename);
    float x1, y1, x2, y2, x3, y3, x4, y4, c1, c2, c3;
    std::string line;

    while (infile >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> c1 >> c2 >> c3) {
        polygon_c_t new_poly = {{{x1,y1}, {x2,y2}, {x3,y3}, {x4,y4}}, {c1, c2, c3}};
        polys.push_back(new_poly);
    }
    return polys;
}

std::vector<rect_c_t> gen_color_rects(std::string filename) {
    // Generate the list of colored rectangles included in the given file
    struct stat file_info;
    if (stat(filename.c_str(), &file_info) != 0) {
        printf("Error: Shapes file '%s' does not exist.\n", filename.c_str());
        exit(1);
    }

    std::vector<rect_c_t> polys;
    std::ifstream infile(filename);
    float x1, y1, x2, y2, x3, y3, x4, y4, c1, c2, c3;
    std::string line;

    while (infile >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> c1 >> c2 >> c3) {
        float width = x2 - x1;
        float height = y4 - y1;
        rect_c_t new_rect = {{x1,y1}, width, height, {c1, c2, c3}};
        polys.push_back(new_rect);
    }
    return polys;
}

std::string float_to_string(float num) {
    std::string str = std::to_string(num);
    str.erase(str.find_last_not_of('0') + 1, std::string::npos);
    str.erase(std::find_if(str.rbegin(), str.rend(), std::bind1st(std::not_equal_to<char>(), '.')).base(), str.end());
    return str;
}

bool point_in_polygon(point_t point, polygon_c_t polygon) {
    bool in = false;
    double px = point.x;
    double py = point.y;

    for (uint8_t i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++) {
        double ix = polygon.points[i].x;
        double iy = polygon.points[i].y;
        double jx = polygon.points[j].x;
        double jy = polygon.points[j].y;

        if( ((iy > py) != (jy > py)) &&
            (px < ((jx - ix) * (py - iy) / (jy - iy)) + ix)) {
            in = !in;
        }
    }
    return in;
}

bool point_in_rect(point_t point, rect_c_t rect) {
    return point.x >= rect.pos.x && point.x <= rect.pos.x + rect.width
            && point.y >= rect.pos.y && point.y <= rect.pos.y + rect.height;
}

bool point_in_circle(point_t point, circle_t circ) {
    double dist = sqrt(pow(point.x - circ.x, 2) + pow(point.y - circ.y, 2));
    return dist < circ.rad;
}