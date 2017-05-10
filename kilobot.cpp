#pragma once
#include "kilolib.h"
#include "vars.cpp"
#include "shapes.cpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

typedef struct neighbor_info_array_t
{
	float measured_distance;
	uint16_t id;
	uint8_t detect_which_feature;
	uint8_t feature_estimate;
	uint32_t time_last_heard_from;
	uint8_t number_of_times_heard_from;
} neighbor_info_array_t;


// Rectangle defining boundary of arena (detected by light change)
double a_w = arena_width - edge_width; double a_h = arena_height - edge_width;
std::vector<point_t> arena_bounds = {{edge_width, edge_width}, {edge_width, a_h}, {a_w, a_h}, {a_w, edge_width}};

static bool point_in_polygon(point_t point, std::vector<point_t> polygon) {
	bool in = false;
	double px = point.x;
	double py = point.y;

	for (uint8_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
		double ix = polygon[i].x;
		double iy = polygon[i].y;
		double jx = polygon[j].x;
		double jy = polygon[j].y;

		if( ((iy > py) != (jy > py)) &&
		    (px < ((jx - ix) * (py - iy) / (jy - iy)) + ix)) {
				in = !in;
		}
	}
	return in;
}

static bool point_in_circle(point_t point, circle_t circ) {
	double dist = sqrt(pow(point.x - circ.x, 2) + pow(point.y - circ.y, 2));
	return dist < circ.rad;
}

class mykilobot : public kilobot {

// Light levels for edge following & color detection
const uint8_t DARK = 0;
const uint8_t GRAY = 1;
const uint8_t LIGHT = 2;
uint16_t curr_light_level;  // DARK, GRAY, or LIGHT. Initialized/detected in setup()
uint16_t edge_thresh = 300;  // Level to differentiate light vs. dark
uint8_t ef_level;  // DARK or LIGHT level stored

// States
const uint8_t SET_LEVELS = 0;
const uint8_t RUN_LOOP = 1;
uint8_t state = RUN_LOOP;





// Feature that this kilobot is observing (may later be changed, but for now its static)
const uint8_t FEATURE_TEMPORAL = 0;  // temporal
const uint8_t FEATURE_COLORS = 1;  // color
const uint8_t FEATURE_CURVATURE = 2;  // curvature
#define NUM_FEATURES 3
//uint8_t detect_which_feature = FEATURE_CURVATURE;  // Set in setup()

// Different search/exploration types
const uint8_t AGENT_REFLECTIVE = 0;
const uint8_t AGENT_LONG_RW = 1;
const uint8_t AGENT_FOLLOW_EDGE = 2;
const uint8_t AGENT_SHORT_RW = 3;
const uint8_t AGENT_RW = 4;
const uint8_t AGENT_TEST = 5;
//uint8_t agent_type = AGENT_FOLLOW_EDGE;





// Test agent parameters
const uint8_t TEST_DEFAULT = 0;
uint8_t test_state = TEST_DEFAULT;

// Edge detection state
const uint8_t EF_INIT = 0;
const uint8_t EF_SEARCH = 1;
const uint8_t EF_FOLLOW = 2;
const uint8_t EF_DISSEMINATE = 3;
uint8_t ef_state = EF_INIT;
bool ef_disseminating_flag = false;
uint32_t ef_last_changed = 0;
const uint8_t TURN_LEFT = 0;
const uint8_t TURN_RIGHT = 1;
const uint8_t TURN_NONE = 2;
uint8_t ef_turn_dir = TURN_LEFT;

// Random walk turn/straight state
const uint8_t RW_INIT = 0;
const uint8_t RW_STRAIGHT = 1;
const uint8_t RW_TURN = 2;
uint8_t rw_state = RW_INIT;
uint32_t rw_last_changed = 0;
uint32_t long_rw_mean_straight_dur = 240 * SECOND;  // kiloticks
uint32_t short_rw_mean_straight_dur = 5 * SECOND;  // kiloticks
uint32_t rw_max_turn_dur = 12 * SECOND;  // kiloticks
// Turn/straight durations are set at the beginning of each transition to that RW state
uint32_t rw_straight_dur;
uint32_t rw_turn_dur;

// Reflective walking (staight + bouncing)
const uint8_t REFLECT_INIT = 0;
const uint8_t REFLECT_STRAIGHT = 1;
uint8_t reflect_state = REFLECT_INIT;

// Bounce off of walls when it hits them (like a screensaver)
const uint8_t BOUNCE = 100;
uint32_t bounce_dur = SECOND * 3;  // kiloticks
uint8_t bounce_turn;

// Feature detection

uint8_t feature_estimate = 127;  // Feature belief 0-255
uint32_t feature_observe_start_time;
bool is_feature_disseminating = false;
uint32_t dissemination_duration_constant = 60 * SECOND;
uint32_t mean_explore_duration = 60 * SECOND;
uint32_t explore_duration;
uint32_t dissemination_duration;
uint32_t dissemination_start_time;
// Accumulating variables for feature detection/determination
uint8_t detect_curvature_dir;
uint32_t curvature_right_dur = 0;
uint32_t curvature_left_dur = 0;
bool is_first_turn = true;
uint32_t num_curv_samples = 0;  // TODO: Temporary for debugging curvature
uint32_t curv_straight_max_thresh = radius * 2 * 10;  // Always straight past 10 body lengths

uint16_t detect_color_level;
uint32_t color_light_dur = 0;
uint32_t color_dark_dur = 0;
uint32_t detect_feature_start_time = 0;
uint32_t detect_level_start_time = 0;
// States of feature observation
const uint8_t DETECT_FEATURE_INIT = 0;  // Begin
const uint8_t DETECT_FEATURE_OBSERVE = 1;  // Continue (includes reset to init)
uint8_t detect_feature_state = DETECT_FEATURE_INIT;
bool is_feature_detect_safe = false;  // Feature detection needs to be enabled in loop

// Temporal pattern detection
std::vector<uint32_t> color_light_durs;
std::vector<uint32_t> color_dark_durs;
const uint32_t max_explore_dur = 60 * SECOND;
const double temporal_std_thresh = 40;

// Beliefs about pattern features start as middle/uncertain (each 127)
// But should be replaced by first message, so starting value likely won't matter much
uint8_t pattern_belief[NUM_FEATURES] = {127, 127, 127};
bool is_updating_belief = false;
const uint8_t DMMD = 0;
const uint8_t DMVD = 1;
uint8_t pattern_decision_method = DMMD;

// Messages/communication
#define NEIGHBOR_INFO_ARRAY_SIZE 20
//uint32_t NEIGHBOR_INFO_ARRAY_TIMEOUT = UINT16_MAX;
uint32_t NEIGHBOR_INFO_ARRAY_TIMEOUT = 120 * SECOND;
message_t rx_message_buffer;
distance_measurement_t rx_distance_buffer;
bool new_message = false;
neighbor_info_array_t neighbor_info_array[NEIGHBOR_INFO_ARRAY_SIZE];
bool neighbor_info_array_locked = false;
bool distance_averaging = false;
uint16_t own_id;
message_t tx_message_data;

// GENERALLY USEFUL FUNCTIONS

uint32_t uniform_rand(uint32_t max_val) {
	// Generate a random int from 0 to max_val
	uint32_t rand_val = (uint32_t)rand_hard() / 255.0 * max_val;
	return rand_val;
}

uint32_t exp_rand(double mean_val) {
	// Generate random value from exponential distribution with mean mean_val
	// According to: http://stackoverflow.com/a/11491526/2552873
	// Generate random float (0,1)
	double unif_val = (double)rand_hard() / 255.0;
	uint32_t exp_val = uint32_t(-log(unif_val) * mean_val);
	return exp_val;
}

uint32_t sum(std::vector<uint32_t> data) {
    // Get the sum of all the elements in the vector
    uint32_t accum = 0;
    for (int i; i<data.size(); ++i) {
        accum += data[i];
    }
    return accum;
}

double mean(std::vector<uint32_t> data) {
    // Calculate the mean of the input data vector
    int accum = 0;
    for (int i=0; i<data.size(); ++i) {
        accum += data[i];
    }
    return (double)accum/data.size();
}

double stddev(std::vector<uint32_t> data) {
    // Calculate standard deviation of input vector
    double m = mean(data);
    double accum = 0.0;
    for (int i=0; i<data.size(); ++i) {
        accum += (data[i] - m)*(data[i] - m);
    }
    return sqrt(accum / data.size());
}

double nanadd(double a, double b) {
    // Add together the two numbers, ignoring nans
    if (std::isnan(a) && std::isnan(b)) {
        return 0;
    } else if (std::isnan(a)) {
        return b;
    } else if (std::isnan(b)) {
        return a;
    } else {
        return a + b;
    }
}

void agent_set_color(rgb color, uint8_t this_agent_type) {
	// Set the color only if the agent type matches what's given
	// Useful for debugging where one agent refers to the movement type of another
	if (this_agent_type == AGENT_RW) {
		agent_set_color(color, AGENT_LONG_RW);
		agent_set_color(color, AGENT_SHORT_RW);
	} else if (this_agent_type == agent_type) {
		set_color(color);
	}
}

uint8_t find_wall_collision() {
    // Use light sensor to detect if outside the black/white area (into gray)
    uint16_t light_level = sample_light();
    if (light_level < 750 && light_level > 250) {
        return 1;
    } else {
        return 0;
    }
}

bool is_random_walk(uint8_t agent_type) {
    return agent_type == AGENT_LONG_RW || agent_type == AGENT_SHORT_RW || agent_type == AGENT_RW;
}

void print_neighbor_info_array() {
	printf("\n\rOwn ID = %x\tPattern beliefs = (%u, %u, %u)\n\r", own_id, pattern_belief[0], pattern_belief[1], pattern_belief[2]);

	printf("\n\rIndex\tID\tFeature\tBelief\tD_meas.\tN_Heard\tTime\n\r");
	for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
		if (neighbor_info_array[i].id != 0) {
			printf("%u\t%x\t%u\t%u\t%u\t%u\t%u\n\r",
                //  1   2   3   4   5   6   7
					i,
					neighbor_info_array[i].id,
                    neighbor_info_array[i].detect_which_feature,
                    neighbor_info_array[i].feature_estimate,
					((uint8_t) neighbor_info_array[i].measured_distance),
					neighbor_info_array[i].number_of_times_heard_from,
					(uint16_t)(kilo_ticks - neighbor_info_array[i].time_last_heard_from));
		}
	}
}

void seed_rng() {
	uint16_t r = kilo_ticks;
	int16_t s;

    // For actual robots
	/*for (uint8_t i = 0; i < 201; ++i) {
		s = get_ambientlight();
		srand(s);
		r += rand();
		s = get_voltage();
		srand(s);
		r += rand();
		s = get_temperature();
		srand(s);
		r += rand();
		delay(1);
	}
	r += kilo_ticks;
	srand(r);
    */

    // For simulator
    srand(id);
}

void initialize_neighbor_info_array() {
	for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
		neighbor_info_array[i].id = 0;
	}
}

void prune_neighbor_info_array() {
	for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
		if (kilo_ticks > (neighbor_info_array[i].time_last_heard_from + NEIGHBOR_INFO_ARRAY_TIMEOUT)) {
			neighbor_info_array[i].id = 0;
		}
	}
}

void update_neighbor_info_array(message_t* m, distance_measurement_t* d) {
	bool can_insert = false;
	bool new_entry;
	uint8_t index_to_insert;

	uint16_t rx_id = (((uint16_t) m->data[0]) << 8) | ((uint16_t) (m->data[1]));
	uint8_t measured_distance_instantaneous = estimate_distance(d);

	for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
		if (neighbor_info_array[i].id == rx_id) {
			can_insert = true;
			new_entry = false;
			index_to_insert = i;
		}
	}

	if (!can_insert) {
		// Data message
		for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
			if (neighbor_info_array[i].id == 0) {
				can_insert = true;
				new_entry = true;
				index_to_insert = i;
				break;
			}
		}
	}

	if (!can_insert) {
		// Data message
		uint8_t largest_measured_distance = 0;
		uint8_t largest_measured_distance_index;
		for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
			if (neighbor_info_array[i].measured_distance > largest_measured_distance) {
				largest_measured_distance = neighbor_info_array[i].measured_distance;
				largest_measured_distance_index = i;
			}
		}
		if (measured_distance_instantaneous < (largest_measured_distance - 5)) {
			// Replace far away entries if table is full and this is closer
			can_insert = true;
			new_entry = true;
			index_to_insert = largest_measured_distance_index;
		}
	}

	if (can_insert) {
		neighbor_info_array[index_to_insert].time_last_heard_from = kilo_ticks;
		if (new_entry) {
			neighbor_info_array[index_to_insert].id = rx_id;
			neighbor_info_array[index_to_insert].measured_distance = measured_distance_instantaneous;
			neighbor_info_array[index_to_insert].number_of_times_heard_from = 0;
		} else {
			if (distance_averaging) {
				neighbor_info_array[index_to_insert].measured_distance *= 0.9;
				neighbor_info_array[index_to_insert].measured_distance += 0.1 * ((float) measured_distance_instantaneous);
			} else {
				neighbor_info_array[index_to_insert].measured_distance = measured_distance_instantaneous;
			}

			if (neighbor_info_array[index_to_insert].number_of_times_heard_from < UINT8_MAX) {
				neighbor_info_array[index_to_insert].number_of_times_heard_from++;
			}
		}
		// Update info whether it's new or not
		neighbor_info_array[index_to_insert].detect_which_feature = m->data[2];
		neighbor_info_array[index_to_insert].feature_estimate = m->data[3];
	}
}


// FEATURE DETECTION FUNCTIONS

void detect_feature_curvature() {
    // Detect duration of time spent turning left vs right

    if (detect_feature_state == DETECT_FEATURE_INIT) {
        if (is_feature_disseminating && kilo_ticks > dissemination_start_time + dissemination_duration) {
            // Check if dissemination time is finished
            is_feature_disseminating = false;
            is_updating_belief = true;  // In loop, update pattern belief using neighbor array info
        } else if (!is_feature_disseminating && curr_light_level != GRAY && is_feature_detect_safe) {
            // Check if in correct movement state for starting observations
            curvature_left_dur = 0;
            curvature_right_dur = 0;
            detect_feature_start_time = kilo_ticks;
            detect_level_start_time = kilo_ticks;
            detect_curvature_dir = ef_turn_dir;
            detect_feature_state = DETECT_FEATURE_OBSERVE;
            is_first_turn = true;
            // TODO: TEMPORARY FOR TESTING! Change back to distribution after
            explore_duration = 3500;
            //explore_duration = exp_rand(mean_explore_duration);
            num_curv_samples = 0;
        }
    } else if (detect_feature_state == DETECT_FEATURE_OBSERVE) {
        // Check for turn direction change
        if (detect_curvature_dir != ef_turn_dir && is_feature_detect_safe) {
            // Add to accumulators if turn direction changes
            num_curv_samples += 1;
            if (is_first_turn) {
                is_first_turn = false;
            } else {
                if (detect_curvature_dir == TURN_LEFT) {
                    curvature_left_dur += kilo_ticks - detect_level_start_time;
                } else if (detect_curvature_dir == TURN_RIGHT) {
                    curvature_right_dur += kilo_ticks - detect_level_start_time;
                }
            }
        }
        if (detect_curvature_dir != ef_turn_dir || !is_feature_detect_safe) {
            // Don't start new observation when in the borderlands
            detect_level_start_time = kilo_ticks;
            detect_curvature_dir = ef_turn_dir;
        } else if (detect_light_level() == GRAY) {
            // Placeholder (won't be saved) until moved out of borderlands
            detect_curvature_dir = TURN_NONE;
        }
        if (!is_feature_detect_safe || kilo_ticks - detect_feature_start_time > explore_duration) {
            double curvature_ratio;
            if (curvature_left_dur > curvature_right_dur) {
                curvature_ratio = (double)curvature_left_dur / curvature_right_dur;
            } else {
                curvature_ratio = (double)curvature_right_dur / curvature_left_dur;
            }
            // Set confidence and estimates
            double confidence;
            double radius_est = pow(1000/(curvature_ratio-1.01), 1.0/1.7);
            if (radius_est >= .5 * curv_straight_max_thresh) {
                confidence = std::min(radius_est / curv_straight_max_thresh, 1.0);
                feature_estimate = 0;
                if (confidence > 0) printf("STRAIGHT (%f, %f)\n", radius_est, confidence);
            } else {
                confidence = 1 - radius_est / curv_straight_max_thresh;
                feature_estimate = 255;
                if (confidence > 0) printf("CURVED   (%f, %f)\n", radius_est, confidence);
            }
            //printf("%f\t%d\t%f\n", curvature_ratio, curvature_right_dur + curvature_left_dur, confidence);

            std::ofstream myfile;
            myfile.open("circle.txt", std::ios::out|std::ios::app);
            myfile << curvature_ratio << "\t" << curvature_left_dur + curvature_right_dur << std::endl;

            detect_feature_state = DETECT_FEATURE_INIT;
            is_feature_disseminating = true;  // Tell message_tx to send updated message
            dissemination_duration = exp_rand(dissemination_duration_constant * confidence);
            dissemination_start_time = kilo_ticks;
        }
    }
}

void detect_feature_temporal() {
    // Determine if there is a temporal pattern using standard deviation of
    // color observation duration

    curr_light_level = detect_light_level();
    if (detect_feature_state == DETECT_FEATURE_INIT) {
        if (is_feature_disseminating && kilo_ticks > dissemination_start_time + dissemination_duration) {
            // Check if dissemination time is finished
            is_feature_disseminating = false;
            is_updating_belief = true;  // In loop, update pattern belief using neighbor array info
        } else if (!is_feature_disseminating && curr_light_level != GRAY && is_feature_detect_safe) {
            // Correct movement state for starting observations
            color_light_durs.clear();
            color_dark_durs.clear();
            color_light_dur = 0;
            color_dark_dur = 0;
            detect_feature_start_time = kilo_ticks;
            detect_level_start_time = kilo_ticks;
            detect_color_level = curr_light_level;
            detect_feature_state = DETECT_FEATURE_OBSERVE;
        }
    } else if (detect_feature_state == DETECT_FEATURE_OBSERVE) {
        // Check for color change
        if (detect_color_level != curr_light_level) {
            // Add to accumulators if light level changes (including to gray)
            if (detect_color_level == LIGHT) {
                color_light_dur += kilo_ticks - detect_level_start_time;
                color_light_durs.push_back(kilo_ticks - detect_level_start_time);
            } else if (detect_color_level == DARK) {
                color_dark_dur += kilo_ticks - detect_level_start_time;
                color_dark_durs.push_back(kilo_ticks - detect_level_start_time);
            }
            detect_color_level = curr_light_level;
            if (curr_light_level != GRAY) {
                // Don't start new observation when in the borderlands
                detect_level_start_time = kilo_ticks;
            }
        }
        if (!is_feature_detect_safe || kilo_ticks - detect_feature_start_time > max_explore_dur) {
            // Feature detection over
            double confidence;
            // Get std for each measurement
            uint32_t total_dur = sum(color_dark_durs) + sum(color_light_durs);
            // Remove first and last elements
            /*if (color_dark_durs.size() > 2) {
                color_dark_durs.pop_back();
                color_dark_durs.erase(color_dark_durs.begin());
            } else {
                color_dark_durs.clear();
            }
            if (color_light_durs.size() > 2) {
                color_light_durs.pop_back();
                color_light_durs.erase(color_light_durs.begin());
            } else {
                color_light_durs.clear();
            }*/

            double color_light_std = stddev(color_light_durs);
            double color_dark_std = stddev(color_dark_durs);
            if (total_dur > 5 * SECOND) {
                // Consider: sum of normalized standard deviations
                double total_std = nanadd(color_light_std/sum(color_light_durs)*100, color_dark_std/sum(color_dark_durs)*100);
                if (total_std == 0) {
                    confidence = 0;
                    feature_estimate = 127;
                } else if (total_std < temporal_std_thresh) {
                    feature_estimate = 255;
                    confidence = 1 - total_std/temporal_std_thresh*.5;
                    printf("%f\n", total_std);
                    //printf("PATTERN!\t(%f)\t%f\n", total_std, confidence);
                } else {
                    feature_estimate = 0;
                    confidence =total_std/70*.5+.5;
                    printf("%f\n", total_std);
                    //printf("no pattern\t(%f)\t%f\n", total_std, confidence);
                    // TODO: fix/approximate maximum
                }
                // If either is 0 duration, assume patterened (only saw 1 color)
            }
            detect_feature_state = DETECT_FEATURE_INIT;
            is_feature_disseminating = true;  // Tell message_tx to send updated message
            dissemination_duration = exp_rand(dissemination_duration_constant * confidence);
            dissemination_start_time = kilo_ticks;
        }
    }
}

void detect_feature_color() {
    // Detect how much time is spent in white vs black
    curr_light_level = detect_light_level();
    if (detect_feature_state == DETECT_FEATURE_INIT) {
        if (is_feature_disseminating && kilo_ticks > dissemination_start_time + dissemination_duration) {
            // Check if dissemination time is finished
            is_feature_disseminating = false;
            is_updating_belief = true;  // In loop, update pattern belief using neighbor array info
        } else if (!is_feature_disseminating && curr_light_level != GRAY && is_feature_detect_safe) {
            // Correct movement state for starting observations
            color_light_dur = 0;
            color_dark_dur = 0;
            detect_feature_start_time = kilo_ticks;
            detect_level_start_time = kilo_ticks;
            detect_color_level = curr_light_level;
            detect_feature_state = DETECT_FEATURE_OBSERVE;
            explore_duration = exp_rand(mean_explore_duration);
        }
    } else if (detect_feature_state == DETECT_FEATURE_OBSERVE) {
        // Check for color change
        if (detect_color_level != curr_light_level || !is_feature_detect_safe || color_light_dur + color_dark_dur + kilo_ticks - detect_level_start_time >= explore_duration) {
            // Add to accumulators if light level changes (including to gray)
            if (detect_color_level == LIGHT) {
                color_light_dur += kilo_ticks - detect_level_start_time;
            } else if (detect_color_level == DARK) {
                color_dark_dur += kilo_ticks - detect_level_start_time;
            }
            detect_color_level = curr_light_level;
            if (curr_light_level != GRAY) {
                // Don't start new observation when in the borderlands
                detect_level_start_time = kilo_ticks;
            }
        }
        if (color_light_dur + color_dark_dur >= explore_duration) {
            double confidence;
            if (color_light_dur > color_dark_dur) {
                //set_color(RGB(0,1,0));
                feature_estimate = 255;
                confidence = (double)color_light_dur / (color_light_dur + color_dark_dur);
                if (confidence > 0) printf("LIGHT    (%f, %f)\n", confidence, confidence);
            } else if (color_light_dur < color_dark_dur) {
                //set_color(RGB(1,.5,0));
                feature_estimate = 0;
                confidence = (double)color_dark_dur / (color_light_dur + color_dark_dur);
                if (confidence > 0) printf("DARK     (%f, %f)\n", 1-confidence, confidence);
            } else {
                //set_color(RGB(1,1,1));
                feature_estimate = 127;
                confidence = 0;
            }
            //printf("%f\n", confidence);
            detect_feature_state = DETECT_FEATURE_INIT;
            is_feature_disseminating = true;  // Tell message_tx to send updated message
            dissemination_duration = exp_rand(dissemination_duration_constant * confidence);
            dissemination_start_time = kilo_ticks;
        }
    }
}

void update_pattern_beliefs() {
    if (is_updating_belief) {
        if (pattern_decision_method == DMMD) {  // Majority-based decisions
            for (uint8_t f = 0; f < NUM_FEATURES; ++f) {
                std::vector<uint8_t> histogram(UINT8_MAX+1, 0);
                for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
                    if (neighbor_info_array[i].id != 0 && neighbor_info_array[i].detect_which_feature == f) {
                        ++histogram[neighbor_info_array[i].feature_estimate];
                    }
                }
                uint8_t new_belief = std::max_element(histogram.begin(), histogram.end()) - histogram.begin();
                if (histogram[new_belief] != 0) {
                    pattern_belief[f] = new_belief;
                }
            }
        } else if (pattern_decision_method == DMVD) {  // Voter-based decisions (lottery)
            std::vector<uint8_t> choices(NEIGHBOR_INFO_ARRAY_SIZE);
            uint8_t num_choices = 0;
            for (uint8_t f = 0; f < NUM_FEATURES; ++f) {
                for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
                    if (neighbor_info_array[i].id != 0 && neighbor_info_array[i].detect_which_feature == f) {
                        choices[num_choices] = neighbor_info_array[i].feature_estimate;
                        num_choices++;
                    }
                }
                // Select from valid indices
                if (num_choices != 0) {
                    int ind = rand_hard() % num_choices;
                    pattern_belief[f] = choices[ind];
                }
            }
        }
    }
}


// AUXILIARY FUNCTIONS FOR LOOP (DETECTION AND MOVEMENT)

uint16_t sample_light() {
    // Light sampling function
	uint16_t num_samples = 0;
	int32_t sum = 0;

	// FOR SIMULATOR:
	// Get point at front/nose of robot
	point_t p;
	double t = pos[2];
	p.x = pos[0] + radius * 1 * cos(t);
	p.y = pos[1] + radius * 1 * sin(t);
	//p.x = pos[0];
    //p.y = pos[1];
    // Check if in arena boundaries. If not, return grey
    if (!point_in_polygon(p, arena_bounds)) {
        // out of arena = GRAY
        return 500;
    } else {
        // Check if in any polygon or circle
        for (int i = 0; i < polygons.size(); i++) {
            std::vector<point_t> poly = polygons[i];
            if (point_in_polygon(p, poly)) {
                return 1000;
            }
        }
        for (int i = 0; i < circles.size(); i++) {
            if (point_in_circle(p, circles[i])) {
                return 1000;
            }
        }
        return 0;
    }

	// FOR ACTUAL ROBOTS:
	/*while (num_samples < 300) {
        int16_t sample = get_ambientlight();
        if (sample != -1) {
                sum += sample;
                num_samples += 1;
        }
   }
   return (sum / num_samples);*/
}

uint8_t detect_light_level() {
    // Detect/return light level (DARK = [0,250), GRAY = [250-750), LIGHT = [750-1024])
    // TODO: Refine these levels/thresholds and turn into constants
    // Get current light level
    uint16_t light = sample_light();
	if (light < 250) {
		return DARK;
	}
	else if (light < 750){
		return GRAY;
	} else {
        return LIGHT;
    }
}

void bounce_init(uint8_t wall_hit) {
	// Start the bounce movement
	// Also set rw_state/ef_state to BOUNCE in the calling function
	// Don't forget to end the bounce phase in the calling function as well...

    // Now it doesn't know what wall it hit. Randomly pick a direction to turn?
    if (rand() % 2 == 0) {
        bounce_turn = TURN_LEFT;
    } else {
        bounce_turn = TURN_RIGHT;
    }
	// Start bounce
	spinup_motors();
	if (bounce_turn == TURN_LEFT) {
		set_motors(kilo_turn_left, 0);
	} else {
		set_motors(0, kilo_turn_right);
	}
}


// LOOP FUNCTIONS FOR EACH AGENT

void random_walk(uint32_t mean_straight_dur, uint32_t max_turn_dur) {
	// Non-blocking random walk, iterating between turning and walking states
	// Durations are in kiloticks

	uint8_t wall_hit = find_wall_collision();
	if (wall_hit != 0 && rw_state != BOUNCE) {
		// Check for wall collision before anything else
		rw_state = BOUNCE;
        if (is_random_walk(agent_type)) {
            is_feature_detect_safe = false;
        }
		bounce_init(wall_hit);
	} else if (rw_state == BOUNCE) {
        uint16_t light_levels = sample_light();
        if (light_levels < 250 || light_levels > 750) {
			// end bounce phase
			rw_state = RW_INIT;
		}
	} else if (rw_state == RW_INIT) {
		// Set up variables
		rw_state = RW_STRAIGHT;
        if (is_random_walk(agent_type)) {
            is_feature_detect_safe = true;
        }
		rw_last_changed = kilo_ticks; rw_straight_dur = exp_rand(mean_straight_dur);
		spinup_motors();
		set_motors(kilo_straight_left, kilo_straight_right);
	} else if (rw_state == RW_STRAIGHT && kilo_ticks > rw_last_changed + rw_straight_dur) {
		// Change to turn state
		rw_last_changed = kilo_ticks;
		rw_state = RW_TURN;
        if (is_random_walk(agent_type)) {
            is_feature_detect_safe = false;
        }
		// Select turning duration in kilo_ticks
		rw_turn_dur = uniform_rand(max_turn_dur);
		// Set turning direction
		spinup_motors();
		bool is_turn_left = rand_hard() & 1;
		if (is_turn_left) {
			set_motors(kilo_turn_left, 0);
		} else { // turn right
			set_motors(0, kilo_turn_left);
		}
	} else if (rw_state == RW_TURN && kilo_ticks > rw_last_changed + rw_turn_dur) {
		// Change to straight state
		rw_last_changed = kilo_ticks;
		rw_state = RW_STRAIGHT;
        if (is_random_walk(agent_type)) {
            is_feature_detect_safe = true;
        }
		// Select staight movement duration
		rw_straight_dur = exp_rand(mean_straight_dur);
		// Set turning direction
		spinup_motors();
		set_motors(kilo_straight_left, kilo_straight_right);
	}
}

void follow_edge() {
	// Non-blocking agent movement design to follow a detected color edge
    // TODO: Remove repeated code (likely by sending the state back to init, which handles everything)

	// Get current light level
    curr_light_level = detect_light_level();
    if (is_feature_disseminating && !ef_disseminating_flag && ef_state != BOUNCE) {
        ef_state = EF_DISSEMINATE;
    } else if (ef_state != EF_DISSEMINATE && ef_state != BOUNCE) {
        ef_disseminating_flag = false;
    }

	// Move accordingly
	uint8_t wall_hit = find_wall_collision();
	if (wall_hit != 0 && ef_state != BOUNCE) {
		// Check for wall collision before anything else
		ef_state = BOUNCE;
        is_feature_detect_safe = false;
		bounce_init(wall_hit);
	} else if (ef_state == BOUNCE) {
        uint16_t light_levels = sample_light();
        if (light_levels < 250 || light_levels > 750) {
			// end bounce phase
			ef_state = EF_INIT;
		}
	} else if (ef_state == EF_INIT) {
		// Store initial light level and start search for edge
		ef_level = curr_light_level;
		ef_state = EF_SEARCH;
        is_feature_detect_safe = false;
		rw_state = RW_INIT;
		random_walk(long_rw_mean_straight_dur, rw_max_turn_dur);
	} else if (ef_state == EF_SEARCH) {
		// Move until edge is detected (light change to opposite state)
		if (curr_light_level != ef_level && curr_light_level != GRAY) {
			// Edge detected! Switch to edge following state and change expected color
			ef_state = EF_FOLLOW;
            is_feature_detect_safe = true;
			ef_level = curr_light_level;
			ef_turn_dir = ef_turn_dir^1;
			spinup_motors();
			if (ef_turn_dir == TURN_LEFT) {
				set_motors(kilo_turn_left, 0);
			} else if (ef_turn_dir == TURN_RIGHT) {
				set_motors(0, kilo_turn_right);
			}
			ef_last_changed = kilo_ticks;
		} else {
			// Keep doing random walk search for an edge
			random_walk(long_rw_mean_straight_dur, rw_max_turn_dur);
		}
	} else if (ef_state == EF_FOLLOW) {
		// EDGE FOLLOWING: When color change is detected, rotate+forward until change is detected, then go the other way.
		if (curr_light_level != ef_level) {
			ef_level = curr_light_level;
			ef_last_changed = kilo_ticks;
			ef_turn_dir = ef_turn_dir^1;
			spinup_motors();
			if (ef_turn_dir == TURN_LEFT) {
				set_motors(kilo_turn_left, 0);
			} else if (ef_turn_dir == TURN_RIGHT) {
				set_motors(0, kilo_turn_right);
			}
		} else if (kilo_ticks > (ef_last_changed + 10*SECOND)) {
			// Edge lost! Search for a new one
			ef_state = EF_INIT;
            is_feature_detect_safe = false;
		}
	} else if (ef_state == EF_DISSEMINATE) {
        // Do random walk while disseminating estimate
        if (!is_feature_disseminating) {
            ef_state = EF_INIT;
            ef_disseminating_flag = false;
        } else if (!ef_disseminating_flag) {
            rw_state = RW_INIT;
            random_walk(long_rw_mean_straight_dur, rw_max_turn_dur);
            ef_disseminating_flag = true;
        }
    }
}

void reflective_walk() {
	// Non-blocking movement. Kilobot walks straight until it hits a wall,
    // then turns and goes straight again until it hits another wall

	uint8_t wall_hit = find_wall_collision();
	if (wall_hit != 0 && reflect_state != BOUNCE) {
		// Check for wall collision before anything else
		reflect_state = BOUNCE;
        is_feature_detect_safe = false;
		bounce_init(wall_hit);
	} else if (reflect_state == BOUNCE) {
        uint16_t light_levels = sample_light();
        if (light_levels < 250 || light_levels > 750) {
			// end bounce phase
			reflect_state = REFLECT_INIT;
		}
	} else if (reflect_state == REFLECT_INIT) {
		// Set up variables
		reflect_state = REFLECT_STRAIGHT;
        is_feature_detect_safe = true;
		spinup_motors();
		set_motors(kilo_straight_left, kilo_straight_right);
	}
}

void test_movement() {
	// Movement/loop contents for test agent
	uint8_t wall_hit = find_wall_collision();
	if (wall_hit != 0 && test_state != BOUNCE) {
		// Check for wall collision before anything else
        test_state = BOUNCE;
		bounce_init(wall_hit);
	} else if (test_state == BOUNCE) {
        uint16_t light_levels = sample_light();
		if (light_levels < 250 || light_levels > 750) {
			// end bounce phase
			test_state = TEST_DEFAULT;
		}
	} else {
		spinup_motors();
        set_motors(kilo_straight_left, kilo_straight_right);
        //set_motors(kilo_turn_left, 0);
        //set_motors(0, kilo_turn_right);
	}
}


// REQUIRED KILOBOT FUNCTIONS

void setup() {

	// put your setup code here, to be run only once
	// Set initial light value
    curr_light_level = detect_light_level();
	rw_last_changed = kilo_ticks;
    // Give them some color so they're visible
    set_color(RGB(1,1,1));
    // Initialize own id
    seed_rng();
	initialize_neighbor_info_array();
	own_id = rand();
	while (own_id == 0 || own_id == 0x1234) {
		own_id = rand();
	}
}

void loop() {
	if (state == RUN_LOOP) {
        // Movement
		if (agent_type == AGENT_FOLLOW_EDGE) {
			follow_edge();
		} else if (agent_type == AGENT_SHORT_RW) {
			random_walk(short_rw_mean_straight_dur, rw_max_turn_dur);
		} else if (agent_type == AGENT_LONG_RW) {
			random_walk(long_rw_mean_straight_dur, rw_max_turn_dur);
		} else if (agent_type == AGENT_REFLECTIVE) {
            reflective_walk();
        } else if (agent_type == AGENT_TEST) {
			test_movement();
		}
        // Feature observation
        if (detect_which_feature == FEATURE_CURVATURE) {
            detect_feature_curvature();
        } else if (detect_which_feature == FEATURE_COLORS) {
            detect_feature_color();
        } else if (detect_which_feature == FEATURE_TEMPORAL) {
            detect_feature_temporal();
        }
        // Neighbor updates
        static uint32_t last_printed;
        neighbor_info_array_locked = true;
        if (new_message == true) {
            update_neighbor_info_array(&rx_message_buffer, &rx_distance_buffer);
            new_message = false;
        }
        prune_neighbor_info_array();
        neighbor_info_array_locked = false;
        if (kilo_ticks > last_printed + 16) {
            //print_neighbor_info_array();
            last_printed = kilo_ticks;
        }
        // Update pattern belief based on neighbor information
        update_pattern_beliefs();

        // Update LED color based on OWN ESTIMATE
        //uint8_t est = feature_estimate/255;
        /*uint8_t est = pattern_belief[detect_which_feature]/255;
        if (detect_which_feature == 0) {
    		set_color(RGB(1, 0, 1-est));
            //set_color(RGB(1, 1-est, 1-est));
            //set_color(RGB(1, est, est));
    	} else if (detect_which_feature == 1) {
    		set_color(RGB(1-est, 1, 0));
            //set_color(RGB(1-est, 1, 1-est));
    	} else if (detect_which_feature == 2) {
    		set_color(RGB(0, 1-est, 1));
            //set_color(RGB(1-est, 1-est, 1));
    	}*/
        // Color full estimates
        if (pattern_belief[1] == 127 || pattern_belief[2] == 127) {
            set_color(RGB(1,1,1));
        } else {
            set_color(RGB(pattern_belief[0] / 255, pattern_belief[1] / 255, pattern_belief[2] / 255));
        }
	}
}


// MESSAGE HANDLING

void update_tx_message_data() {
	tx_message_data.type = NORMAL;
	// ID
	tx_message_data.data[0] = ((uint8_t) ((own_id & 0xff00) >> 8));
	tx_message_data.data[1] = ((uint8_t) (own_id & 0x00ff));
	// Feature variable measured
	tx_message_data.data[2] = detect_which_feature;
	// Belief about feature
	tx_message_data.data[3] = feature_estimate;
	tx_message_data.crc = message_crc(&tx_message_data);
}

// Send message (continuously)
message_t *message_tx() {
    // Message should be changed/set by respective detection/observation function
    if (is_feature_disseminating) {
        update_tx_message_data();
	       return &tx_message_data;
    } else {
        return NULL;
    }
}

// Receive message
void message_rx(message_t *m, distance_measurement_t *d) {
    if (neighbor_info_array_locked == false) {
		rx_message_buffer = (*m);
		rx_distance_buffer = (*d);
		new_message = true;
	}
}

// Executed on successful message send
void message_tx_success() {}

};
