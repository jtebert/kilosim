#pragma once
#include "kilolib.h"
#include "vars.h"
#include "shapes.h"
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
polygon_t arena_bounds = {
        {{edge_width, edge_width}, {edge_width, a_h}, {a_w, a_h}, {a_w, edge_width}},
        {0,0,0}};

static bool point_in_polygon(point_t point, polygon_t polygon) {
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

static bool point_in_circle(point_t point, circle_t circ) {
	double dist = sqrt(pow(point.x - circ.x, 2) + pow(point.y - circ.y, 2));
	return dist < circ.rad;
}

class mykilobot : public kilobot {

// Light levels for edge following & color detection
const uint8_t DARK = 0;
const uint8_t GRAY = 1;
const uint8_t LIGHT = 2;
uint8_t curr_light_level;  // DARK, GRAY, or LIGHT. Initialized/detected in setup()
uint16_t edge_thresh = 300;  // Level to differentiate light vs. dark
uint8_t ef_level;  // DARK or LIGHT level stored

// States
const uint8_t SET_LEVELS = 0;
const uint8_t RUN_LOOP = 1;
uint8_t state = RUN_LOOP;





// Feature that this kilobot is observing (may later be changed, but for now its static)
const uint8_t RED = 0;
const uint8_t GREEN = 1;
const uint8_t BLUE = 2;

// Different search/exploration types
const uint8_t AGENT_LONG_RW = 1;
const uint8_t AGENT_SHORT_RW = 3;
const uint8_t AGENT_RW = 4;



const uint8_t TURN_LEFT = 0;
const uint8_t TURN_RIGHT = 1;

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

const char* feature_to_color() {
	char const* ch = new char;
	if (detect_which_feature == 0) {
		ch = "R";
	} else if (detect_which_feature == 1) {
		ch = "G";
	} else {
		ch = "B";
	}
	return ch;
}

uint8_t find_wall_collision() {
    // Use light sensor to detect if outside the black/white area (into gray)
    uint8_t light_level = detect_light_level(detect_which_feature);
    if (light_level == GRAY) {
        return 1;
    } else {
        return 0;
    }
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

void detect_feature_color() {
    // Detect how much time is spent in white vs black
	//printf("%d\n", is_feature_detect_safe);
    curr_light_level = detect_light_level(detect_which_feature);
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
                //if (confidence > 0) printf("%s LIGHT    (%f, %f)\n", feature_to_color(), confidence, confidence);
            } else if (color_light_dur < color_dark_dur) {
                //set_color(RGB(1,.5,0));
                feature_estimate = 0;
                confidence = (double)color_dark_dur / (color_light_dur + color_dark_dur);
                //if (confidence > 0) printf("%s DARK     (%f, %f)\n", feature_to_color(), 1-confidence, confidence);
            } else {
                //set_color(RGB(1,1,1));
                feature_estimate = 127;
                confidence = 0;
            }
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
                // Else: heard from no one; keep current belief for feature
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

std::vector<uint16_t> sample_light() {
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
        return {500, 500, 500};
    } else {
        // Check if in any polygon or circle
        for (int i = 0; i < polygons.size(); i++) {
            polygon_t poly = polygons[i];
            if (point_in_polygon(p, poly)) {
                return {uint16_t(poly.color[0]*1024),
						uint16_t(poly.color[1]*1024),
						uint16_t(poly.color[2]*1024)};
            }
        }
        for (int i = 0; i < circles.size(); i++) {
            if (point_in_circle(p, circles[i])) {
                return {1000, 1000, 1000};
            }
        }
        return {0, 0, 0};
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

uint8_t detect_light_level(uint8_t feature_ind) {
    // Detect/return light level (DARK = [0,250), GRAY = [250-750), LIGHT = [750-1024])
    // TODO: Refine these levels/thresholds and turn into constants
    // Get current light level
    std::vector<uint16_t> light = sample_light();
	if (light[feature_ind] < 250) {
		return DARK;
	}
	else if (light[feature_ind] < 750){
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
		is_feature_detect_safe = false;
		bounce_init(wall_hit);
	} else if (rw_state == BOUNCE) {
        std::vector<uint16_t> light_levels = sample_light();
        if (light_levels[0] < 250 || light_levels[0] > 750) {
			// end bounce phase
			rw_state = RW_INIT;
		}
	} else if (rw_state == RW_INIT) {
		// Set up variables
		rw_state = RW_STRAIGHT;
		is_feature_detect_safe = true;
		rw_last_changed = kilo_ticks; rw_straight_dur = exp_rand(mean_straight_dur);
		spinup_motors();
		set_motors(kilo_straight_left, kilo_straight_right);
	} else if (rw_state == RW_STRAIGHT && kilo_ticks > rw_last_changed + rw_straight_dur) {
		// Change to turn state
		rw_last_changed = kilo_ticks;
		rw_state = RW_TURN;
		is_feature_detect_safe = false;
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
		is_feature_detect_safe = true;
		// Select staight movement duration
		rw_straight_dur = exp_rand(mean_straight_dur);
		// Set turning direction
		spinup_motors();
		set_motors(kilo_straight_left, kilo_straight_right);
	}
}


// REQUIRED KILOBOT FUNCTIONS

void setup() {

	// put your setup code here, to be run only once
	// Set initial light value
    curr_light_level = detect_light_level(detect_which_feature);
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
        random_walk(long_rw_mean_straight_dur, rw_max_turn_dur);

        // Feature observation
        detect_feature_color();

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
