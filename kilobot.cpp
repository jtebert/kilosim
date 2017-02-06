#pragma once
#include "kilolib.h"
#include "vars.cpp"
#include <iostream>
#include <vector>
#include <algorithm>

typedef struct point_t {
		double x;
		double y;
} point_t;

typedef struct circle_t {
		// x, y define center of circle
		double x;
		double y;
		double rad;
} circle_t;

typedef struct neighbor_info_array_t
{
	float measured_distance;
	uint16_t id;
	uint8_t detect_which_feature;
	uint8_t feature_belief;
	uint32_t time_last_heard_from;
	uint8_t number_of_times_heard_from;
} neighbor_info_array_t;


// Shapes for edge following
std::vector<point_t> square = {{250, 1300}, {750, 1300}, {750, 800}, {250, 800}};
std::vector<point_t> triangle = {{300, 1600}, {1100, 1600}, {700, 2300}};
std::vector<point_t> stripe1 = {{1600, 2400}, {1800, 2400}, {1800, 0}, {1600, 0}};
std::vector<point_t> stripe2 = {{2000, 2400}, {2200, 2400}, {2200, 0}, {2000, 0}};
std::vector<std::vector<point_t> > polygons = {square, triangle, stripe1, stripe2};
circle_t circle1 = {1100, 500, 300};
circle_t circle2 = {400, 450, 150};
std::vector<circle_t> circles = {circle1, circle2};

// Rectangle defining boundary of arena (detected by light change)
double edge_width = 32;
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
uint16_t edge_thresh = 300;         // Level to differentiate light vs. dark
uint8_t ef_level;          // DARK or LIGHT level stored

// States
const uint8_t SET_LEVELS = 0;
const uint8_t RUN_LOOP = 1;
uint8_t state = RUN_LOOP;

// Different search/exploration types
const uint8_t AGENT_FOLLOW_EDGE = 0;
const uint8_t AGENT_SHORT_RW = 1;
const uint8_t AGENT_LONG_RW = 2;
const uint8_t AGENT_RW = 3;
const uint8_t AGENT_TEST = 4;
uint8_t agent_type = AGENT_FOLLOW_EDGE;

// Test agent parameters
const uint8_t TEST_DEFAULT = 0;
uint8_t test_state = TEST_DEFAULT;

// Edge detection state
const uint8_t EF_INIT = 0;
const uint8_t EF_SEARCH = 1;
const uint8_t EF_FOLLOW = 2;
uint8_t ef_state = EF_INIT;
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
// Turn/straight durations are set at the beginning of each transition to that RW state
uint32_t rw_straight_dur;
uint32_t rw_turn_dur;

// Bounce off of walls when it hits them (like a screensaver)
const uint8_t BOUNCE = 100;
uint32_t bounce_dur = SECOND * 3;  // kiloticks
uint8_t bounce_turn;

// Feature detection
// Feature that this kilobot is observing (may later be changed, but for now its static)
const uint8_t FEATURE_TEMPORAL = 0;  // temporal
const uint8_t FEATURE_CURVATURE = 1;  // curvature
const uint8_t FEATURE_COLORS = 2;  // color
#define NUM_FEATURES 3
uint8_t detect_which_feature = FEATURE_COLORS;  // Set in setup()

uint8_t feature_belief = 127;  // Feature belief 0-255
uint32_t feature_observe_start_time;
bool is_feature_disseminating = false;
uint32_t dissemination_duration_constant = 10 * SECOND;
// TODO: Adjust max duration
uint32_t mean_explore_duration = 10 * SECOND;
uint32_t explore_duration;
uint32_t dissemination_duration;
uint32_t dissemination_start_time;
// Accumulating variables for feature detection/determination
uint8_t detect_curvature_dir;
uint32_t curvature_right_dur = 0;
uint32_t curvature_left_dur = 0;
double curvature_ratio_thresh = 1.1;
bool is_first_turn = true;

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

// Pattern that the bot thinks it sees (confidence based on what is consistent with its feature observations)
// zB: curvature is consistent with stripes or rings; temporal is stripes or rings
const uint8_t PATTERN_STRIPES = 0;
const uint8_t PATTERN_DOTS = 1;
const uint8_t PATTERN_RINGS = 2;
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
uint32_t NEIGHBOR_INFO_ARRAY_TIMEOUT = 60 * SECOND;
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

uint32_t exp_rand(uint32_t mean_val) {
	// Generate random value from exponential distribution with mean mean_val
	// According to: http://stackoverflow.com/a/11491526/2552873
	// Generate random float (0,1)
	double unif_val = (double)rand_hard() / 255.0;
	uint32_t exp_val = -log(unif_val) * mean_val;
	return exp_val;
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
                    neighbor_info_array[i].feature_belief,
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

	if (can_insert == false) {
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

	if (can_insert == false) {
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

	if (can_insert == true) {
		neighbor_info_array[index_to_insert].time_last_heard_from = kilo_ticks;
		if (new_entry == true) {
			neighbor_info_array[index_to_insert].id = rx_id;
			neighbor_info_array[index_to_insert].measured_distance = measured_distance_instantaneous;
			neighbor_info_array[index_to_insert].number_of_times_heard_from = 0;
		} else {
			if (distance_averaging == true) {
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
		neighbor_info_array[index_to_insert].feature_belief = m->data[3];
	}
}


// FEATURE DETECTION FUNCTIONS

void detect_feature_curvature() {
    // Detect duration of time spent turning left vs right
    // TODO: If it loses edge, make sure time spent spinning isn't included in the left/right duration accumulators

    if (detect_feature_state == DETECT_FEATURE_INIT) {
        //set_color(RGB(0,0,1));
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
        }
    } else if (detect_feature_state == DETECT_FEATURE_OBSERVE) {
        // Check for turn direction change
        if (detect_curvature_dir != ef_turn_dir && is_feature_detect_safe) {
            // Add to accumulators if turn direction changes
            // If edge is lost or enter borderlands, don't record this turn
            // TODO: Don't count first turn (alignment)
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
            // If in borderlands, it will check each tick if it's moved out,
            // then start observing if it has
            detect_curvature_dir = TURN_NONE;
        }
        if (!is_feature_detect_safe || kilo_ticks - detect_feature_start_time > explore_duration) {
            // TODO: Replace explore_duration
            double curvature_ratio;
            double confidence;
            if (curvature_left_dur > curvature_right_dur) {
                curvature_ratio = (double)curvature_left_dur / curvature_right_dur;
                confidence = (double)curvature_left_dur / (color_light_dur + curvature_right_dur);
            } else {
                curvature_ratio = (double)curvature_right_dur / curvature_left_dur;
                confidence = (double)curvature_right_dur / (curvature_left_dur + curvature_right_dur);
            }
            printf("%d\t%d\t%f\n", curvature_left_dur, curvature_right_dur, curvature_ratio);
            if (curvature_ratio >= curvature_ratio_thresh) {
                feature_belief = 255;
                set_color(RGB(1,0,1));
            } else {
                feature_belief = 0;
                set_color(RGB(1,1,1));
                // TODO: Set confidence different than ratio (otherwise biases toward curvature)
            }
            // Not safe = observation period ended. Return to init state until
            // robot says it's safe to start observing again
            detect_feature_state = DETECT_FEATURE_INIT;
            // Update message that's being sent (message_tx should take care of the rest)
            //out_message.data[2] = feature_belief;
            // Prevent the current/new feature belief from being overwritten (some sort of flag or another detect_feature_state?)
            is_feature_disseminating = true;  // Tell message_tx to send updated message
            // Determine dissemination duration (exponentially related to confidence in belief: mean ratio of light vs dark * some constant)
            dissemination_duration = exp_rand(dissemination_duration_constant * confidence);
            dissemination_start_time = kilo_ticks;
        }
    }
}

void detect_feature_temporal() {
    // TODO: Detect pattern of block vs white observed
    // What am I actually calculating? STD/variability of time spent in each? (consistency?)
    // We'll save this one until later...
    // Need to store:
    // - Array/vector of different times spent in black/white? (??)
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
        /*if (detect_color_level != curr_light_level || !is_feature_detect_safe || kilo_ticks- detect_feature_start_time >= max_explore_dur) {*/
        if (detect_color_level != curr_light_level || !is_feature_detect_safe || color_light_dur + color_dark_dur + kilo_ticks - detect_level_start_time >= explore_duration) {
            // Add to accumulators if light level changes (including to gray)
            // OR if robot is no longer "safe" to observe features
            if (detect_color_level == LIGHT) {
                color_light_dur += kilo_ticks - detect_level_start_time;
            } else if (detect_color_level == DARK) {
                color_dark_dur += kilo_ticks - detect_level_start_time;
            }
            detect_color_level = curr_light_level;
            if (curr_light_level != GRAY) {
                // Don't start new observation when in the borderlands
                detect_level_start_time = kilo_ticks;
                // If in borderlands, it will check each tick if it's moved out,
                // then start observing if it has
            }
        }
        /*if (!is_feature_detect_safe || kilo_ticks - detect_feature_start_time > max_explore_dur) {*/
        if (color_light_dur + color_dark_dur >= explore_duration) {
            double confidence;
            // TODO: Also make confidence related to duration of observation?
            if (color_light_dur > color_dark_dur) {
                //set_color(RGB(0,1,0));
                feature_belief = 255;
                confidence = (double)color_light_dur / (color_light_dur + color_dark_dur);
            } else if (color_light_dur < color_dark_dur) {
                //set_color(RGB(1,.5,0));
                feature_belief = 0;
                confidence = (double)color_dark_dur / (color_light_dur + color_dark_dur);
            } else {
                //set_color(RGB(1,1,1));
                feature_belief = 127;
                confidence = 0;
            }
            // Not safe = observation period ended. Return to init state until
            // robot says it's safe to start observing again
            detect_feature_state = DETECT_FEATURE_INIT;
            // Update message that's being sent (message_tx should take care of the rest)
            //out_message.data[2] = feature_belief;
            // Prevent the current/new feature belief from being overwritten (some sort of flag or another detect_feature_state?)
            is_feature_disseminating = true;  // Tell message_tx to send updated message
            // Determine dissemination duration (exponentially related to confidence in belief: mean ratio of light vs dark * some constant)
            dissemination_duration = exp_rand(dissemination_duration_constant * confidence);
            dissemination_start_time = kilo_ticks;
        }
    }
}

void update_pattern_beliefs() {
    // TODO: Update pattern beliefs based on the contents of the neighbor array
    if (is_updating_belief) {
        // Get all neighbors detected for each pattern
         std::vector<std::vector<uint8_t>> neighbor_feature_beliefs(
                 NUM_FEATURES,
                 std::vector<uint8_t>(NEIGHBOR_INFO_ARRAY_SIZE, 0));
        uint8_t neighbor_counts[NUM_FEATURES] = {0};

        for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
            uint8_t f = neighbor_info_array[i].detect_which_feature;
            neighbor_feature_beliefs[f][neighbor_counts[f]] = neighbor_info_array[i].feature_belief;
            neighbor_counts[f] += 1;
        }
        if (pattern_decision_method == DMMD) {  // Majority-based decisions
            for (uint8_t f = 0; f < NUM_FEATURES; ++f) {
                std::vector<uint8_t> histogram(UINT8_MAX+1, 0);
                for (uint8_t i = 0; i < neighbor_counts[f]; ++i) {
                    ++histogram[neighbor_feature_beliefs[f][i]];
                }
                pattern_belief[f] = std::max_element(histogram.begin(), histogram.end()) - histogram.begin();
            }
        } else if (pattern_decision_method == DMVD) {  // Voter-based decisions
            for (uint8_t f = 0; f < NUM_FEATURES; ++f) {
                int ind = rand_hard() % neighbor_counts[f];
                pattern_belief[f] = neighbor_feature_beliefs[f][ind];
            }
        }
        // Use either DMMD or DMVD (probably set a variable somewhere for that)
        // Set belief values in array accordingly
        is_updating_belief = false;
        if (pattern_belief[2] < 127) {
            set_color(RGB(1, .5, 0.));
        } else if (pattern_belief[2] > 127) {
            set_color(RGB(0,1,0));
        } else {
            set_color(RGB(1,1,1));
        }
    }
}


// AUXILIARY FUNCTIONS FOR LOOP (DETECTION AND MOVEMENT)

int16_t sample_light() {
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
    if (point_in_polygon(p, arena_bounds)) {
        // Check if in any polygon or circle
        int16_t is_in_shape;
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
        return 0; // if it didn't already return a value, it's not in any of the shapes
    } else {
        // out of arena
        return 500;
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
		random_walk(40 * SECOND, 4.5 * SECOND);
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
			// TODO: Doesn't have to be left, picked arbitrarily
		} else {
			// Keep doing random walk search for an edge
			random_walk(40 * SECOND, 4.5 * SECOND);
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
        //set_motors(kilo_straight_left, kilo_straight_right);
        set_motors(kilo_turn_left, 0);
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
			random_walk(5 * SECOND, 4.5 * SECOND);
		} else if (agent_type == AGENT_LONG_RW) {
			random_walk(40 * SECOND, 4.5 * SECOND);
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
	tx_message_data.data[3] = feature_belief;

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
    // Extract relevant data
    //uint8_t feature_ind = message->data[1];
    //uint8_t new_feature_belief_val = message->data[2];
    // TODO: implement belief update rule (likely DMMD or DMVD)
    // Create new average and update pattern belief
    // uint8_t new_avg = ((uint32_t)pattern_belief[feature_ind] * message_count[feature_ind] + new_feature_belief_val) / (message_count[feature_ind] + 1);
    // pattern_belief[feature_ind] = new_avg;
    // TODO: Update neighbor table (will get starting code from Melvin)
}

// Executed on successful message send
void message_tx_success() {}

};
