//
// Created by jtebert on 5/24/17.
//

#include "vars.h"

int trial_num = 0;
int num_robots = 120;
uint8_t num_features = 3;
std::vector<uint8_t> use_features = {0, 1, 2};

// Logging results
bool log_debug_info = true;
std::string log_file_name_base = "simulation-";
std::string log_file_dir = "logs";

// Arena dimensions
double edge_width = 48;  // mm
int arena_width = 2400;  // mm
int arena_height = 2400;  // mm

// Arena parameters for shapes
int arena_rows = 10;
float color_fill_ratio[3] = {0.3, 0.2, 0.8};
std::string shapes_filename_base = "shapes-";
std::string shapes_dir = "shapes";
std::vector<polygon_c_t> polygons = {};
std::vector<rect_c_t> rects = {};
std::vector<circle_t> circles = {};