#ifndef VARS_CPP
#define VARS_CPP
#include "shapes.h"

int trial_num = 0;

const uint32_t buffer_size = 1000000;
const uint8_t channels = 2;
const int delay_init = 0; //delay between time steps, use if program is too fast
const uint32_t windowWidth = 1000;  //display window
const uint32_t windowHeight = 1000;  //display window
const uint32_t comm_noise_std = 5; //standard dev. of sensor noise
//#define PI 3.14159265358979324
const double TWO_PI = 6.28318530717958648;
const double p_control_execute = .99;  // probability of a controller executing its time step
const uint8_t SKIPFRAMES = 0;
const int shuffles = 20;
const int circledef = 30;

double edge_width = 48;

int arena_width = 2400;  // mm
int arena_height = 2400;  // mm

// Arena parameters for shapes
int arena_rows = 10;
float color_fill_ratio[3] = {0.3, 0.2, 0.8};
std::string shapes_filename_base = "shapes-";
std::string shapes_dir = "shapes";
std::vector<polygon_t> polygons;
std::vector<circle_t> circles = {};

#endif