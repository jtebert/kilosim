#ifndef VARS_H
#define VARS_H

#include <vector>
#include "shapes.h"

// General parameters
extern int num_threads; // Number of CPU threads to parallelize across
extern int trial_num;
extern int num_robots;
extern int timelimit;
extern uint8_t num_features;
extern std::vector<int> use_features;
extern bool showscene;

// Communication & dissemination
extern float comm_dist;        // (mm) Maximum distance kilobots can communicate
extern bool exp_observation;   // Use exponential distribution for observation duration (true) or constant (false)
extern bool exp_dissemination; // Use exponential distribution for dissemination duration (true) or constant (false)
extern bool use_confidence;    // Use confidence to determine dissemination duration or not
extern bool allow_retransmit;  // Allow re-transmission of messages in neighbor array
//extern uint32_t num_retransmit;  // Max. num. neighbors from table allowed to be retransmitted in each dissem. period
extern uint8_t comm_rate;              // Number of ticks per message
extern uint8_t belief_update_strategy; // 0=none, 1=DMMD, 2 = DMVD

// Logging results
extern bool log_debug_info;
extern std::string log_file_dir;
extern std::string log_filename_base;
extern std::string log_filename;
extern std::string decision_filename_base;
extern std::string decision_filename;
extern std::string comm_filename_base;
extern std::string comm_log_filename;
extern std::string params_filename_base;
extern std::string params_filename;

// Arena dimensions
extern double edge_width; // mm
extern int arena_width;   // mm
extern int arena_height;  // mm

// Arena parameters for shapes
extern int arena_rows;
extern float color_fill_ratio[3];
extern std::string circles_filename_base;
extern int circles_radius;
extern std::string rects_filename_base;
extern std::string polys_filename_base;
extern std::string shapes_dir;
extern std::vector<polygon_c_t> polygons;
extern std::vector<circle_t> circles;
extern std::vector<rect_c_t> rects;
extern rect_c_t arena_bounds;

// Constants
const uint32_t buffer_size = 1000000;
const uint8_t channels = 2;
const int delay_init = 0;           //delay between time steps, use if program is too fast
const uint32_t windowWidth = 1000;  //display window
const uint32_t windowHeight = 1000; //display window
const uint32_t comm_noise_std = 5;  //standard dev. of sensor noise
const double TWO_PI = 6.28318530717958648;
const double p_control_execute = .99; // probability of a controller executing its time step
const uint8_t SKIPFRAMES = 0;
const int shuffles = 20;
const int circledef = 30;
const uint8_t SECOND = 32;

// Time constants for detection
extern uint32_t dissemination_duration_constant;
extern uint32_t mean_explore_duration;
extern uint32_t neighbor_info_array_timeout;

// Diffusion parameters
extern float diffusion_constant;         // (D) (0-1) influence of concentration messages on own concentration
extern float diffusion_decision_thresh;  // as difference from 0/1
extern uint32_t diffusion_decision_time; // (kiloticks) time past threshold before committing to decision

// Allocating agents to features
extern bool dynamic_allocation;                  // Whether agents can change which feature they detect
extern std::vector<double> initial_distribution; // Set initial distribution of agents between features
extern uint8_t feature_switch_when;              // 0=switch after decision (default); 1=switch for each observation
extern uint8_t feature_switch_to;                // switch to feature with concentration: 0=closest to 0.5; 1=furthest from 0.5

// Which feature set to use (color or monochrome)
extern uint8_t which_feature_set; // 0=monochrome, 1=color

#endif