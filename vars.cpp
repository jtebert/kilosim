//
// Created by jtebert on 5/24/17.
//

#include "vars.h"

// General parameters
int num_threads = 4;  // Number of CPU threads to parallelize across
int trial_num = 0;
int num_robots = 120;
int timelimit = 180 * 60;
uint8_t num_features = 3;
std::vector<uint8_t> use_features = {0, 1, 2};
bool showscene = true;

// Communication & dissemination
float comm_dist = 6 * 16;  // 3 body-lengths
bool exp_observation = true;  // Use exponential distribution for observation duration (true) or constant (false)
bool exp_dissemination = true;
bool use_confidence = true;
bool allow_retransmit = false; // Allow re-transmission of messages in neighbor array
//uint32_t num_retransmit = 500;  // Max. num. neighbors from table allowed to be retransmitted in each dissem. period
uint8_t comm_rate = 3;  // Number of ticks per message

// Logging results
bool log_debug_info = true;
std::string log_file_dir = "logs";
std::string log_filename_base = "simulation-";
std::string decision_filename_base = "decision-";
std::string comm_filename_base = "communication-";
std::string params_filename_base = "params-";

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
// Rectangle defining boundary of arena (detected by light change)
rect_c_t arena_bounds = {{edge_width, edge_width}, (float)(arena_width - 2*edge_width), (float)(arena_height - 2*edge_width), {0,0,0}};

// Time constants for detection
uint32_t dissemination_duration_constant = 60 * SECOND;
uint32_t mean_explore_duration = 60 * SECOND;
uint32_t neighbor_info_array_timeout = 120 * SECOND;

// Diffusion parameters
float diffusion_constant = 0.1;  // (D) (0-1) influence of belief messages on own concentration
float diffusion_decision_thresh = 0.1;  // as difference from 0/1
uint32_t diffusion_decision_time = 30 * SECOND;  // (kiloticks) time past threshold before committing to decision

