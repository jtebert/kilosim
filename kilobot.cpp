#pragma once

#include "kilolib.h"
#include "vars.h"
//#include "shapes.h"
#include <iostream>
#include <fstream>
#include <algorithm>

typedef struct neighbor_info_array_t
{
	float measured_distance;
	uint16_t id;
	uint8_t detect_which_feature;
	uint8_t feature_estimate;
	uint32_t time_last_heard_from;
    uint32_t diffusion_timer;
	uint8_t number_of_times_heard_from;
} neighbor_info_array_t;


class mykilobot : public kilobot {

// TODO: DIFFUSION PARAMETERS
float concentrations[3] = {0.5, 0.5, 0.5};
bool decision_locked[3] = {false, false, false};  // Has a decision been made
//uint8_t decision[3];  // What value was decided for each feature?
uint32_t decision_timer[3] = {0,0,0};  // How long has it been past the threshold?

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
uint8_t bounce_turn;

// Retransmission of messages
bool is_retransmit = false;  // Alternate between transmitting own message and re-transmitting random neighbor
std::vector<message_t> retransmit_messages;  // List of messages that will be retransmitted in each dissemination period


// Feature detection

uint8_t feature_estimate = 127;  // Feature belief 0-255
uint32_t feature_observe_start_time;
bool is_feature_disseminating = false;
//uint32_t dissemination_duration_constant = 60 * SECOND;
//uint32_t mean_explore_duration = 60 * SECOND;
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
#define NEIGHBOR_INFO_ARRAY_SIZE 100
//uint32_t neighbor_info_array_timeout = UINT16_MAX;
//uint32_t neighbor_info_array_timeout = 120 * SECOND;
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

uint16_t count_neighbors() {
    // Count how many neighbors in neighbor info array (how many with non-zero ID)
    uint16_t num_neighbors = 0;
    for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; i++) {
        if (neighbor_info_array[i].id != 0 && neighbor_info_array[i].detect_which_feature == detect_which_feature) {
            num_neighbors++;
        }
    }
    return num_neighbors;
}

std::vector<uint32_t> mult_rand_int(uint32_t max_val, uint32_t num_vals) {
    // Generate multiple (num_vals) random values from 0 to max_val (inclusive, I think)

    std::vector<uint32_t> out_vals(num_vals);

    if (num_vals < max_val) {
        /*if (num_vals > 100) {
            // Better to generate and check for duplicates
            std::default_random_engine generator;
            std::uniform_int_distribution<uint32_t> distribution(0, max_val);
            for (int i = 0; i < num_vals; i++) {
                uint32_t new_val = distribution(generator);
                while (std::find(out_vals.begin(), out_vals.end(), new_val) != out_vals.end()) {
                    new_val = distribution(generator);
                }
                out_vals[i] = new_val;
            }
            return out_vals;
        } else {*/
            // Better to generate all the values and pick from them
            // List all possible integer values
            std::vector<uint32_t> init_all_vals(max_val);
            std::iota(std::begin(init_all_vals), std::end(init_all_vals), 0);
            for (int i = 0; i < num_vals; i++) {
                // Pick one randomly by index and add to out_vals
                uint32_t use_ind = uniform_rand(init_all_vals.size());
                out_vals[i] = init_all_vals[use_ind];
                // Remove the selected number from the base list
                init_all_vals.erase(init_all_vals.begin() + use_ind);
            }
            return out_vals;
        //}
    } else {
        // Not enough values. Return what you have.
        // List all possible integer values
        std::vector<uint32_t> init_all_vals(max_val);
        std::iota(std::begin(init_all_vals), std::end(init_all_vals), 0);
        return init_all_vals;
    }

    /* // VERSION 2

    }*/
}

std::vector<message_t> generate_retransmit_messages(std::vector<uint32_t> neighbor_inds) {
    // Generate a list of neighbor messages for kilobot to retransmit
    std::vector<message_t> messages(neighbor_inds.size());
    if (neighbor_inds.size() > 0) {
        for (int i = 0; i < neighbor_inds.size(); i++) {
            neighbor_info_array_t neighbor = neighbor_info_array[neighbor_inds[i]];
            message_t message_new;
            message_new.type = NORMAL;
            // ID
            message_new.data[0] = ((uint8_t) ((neighbor.id & 0xff00) >> 8));
            message_new.data[1] = ((uint8_t) (neighbor.id & 0x00ff));
            // Feature variable measured
            message_new.data[2] = neighbor.detect_which_feature;
            // Estimate of feature
            message_new.data[3] = neighbor.feature_estimate;
            message_new.crc = message_crc(&tx_message_data);
            messages[i] = message_new;
        }
    }
    return messages;
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
    for (int i=0; i<data.size(); ++i) {
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
	printf("\n\n\nOwn ID = %d\tPattern beliefs = (%u, %u, %u)\tConcentrations = (%f, %f, %f)\n",
           id, pattern_belief[0], pattern_belief[1], pattern_belief[2],
           concentrations[0], concentrations[1], concentrations[2]);

	printf("Index\tID\tFeature\tBelief\tD_meas.\tN_Heard\tTime\n\r");
	for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
		if (neighbor_info_array[i].id != 0) {
			printf("%u\t%d\t%u\t%u\t%u\t%u\t%u\n\r",
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
		if (kilo_ticks > (neighbor_info_array[i].time_last_heard_from + neighbor_info_array_timeout)) {
			neighbor_info_array[i].id = 0;
		}
	}
}

uint8_t* float_to_bytes(float f) {
    uint8_t *out = reinterpret_cast<uint8_t*>(&f);
    return out;
}

float bytes_to_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    // Convert the 4 uint8_t to a 32-bit float
    union {
        float f;
        uint8_t b[4];
    } u;
    u.b[0] = b0;
    u.b[1] = b1;
    u.b[2] = b2;
    u.b[3] = b3;
    return u.f;
}

uint8_t* uint16_to_bytes(uint16_t n) {
    uint8_t *out = reinterpret_cast<uint8_t*>(&n);
    return out;
}

uint16_t bytes_to_uint16(uint8_t b0, uint8_t b1) {
    union {
        uint16_t n;
        uint8_t b[2];
    } u;
    u.b[0] = b0;
    u.b[1] = b1;
    return u.n;
}

void update_neighbor_info_array(message_t* m, distance_measurement_t* d) {
	bool can_insert = false;
	bool new_entry;
	uint8_t index_to_insert;

    //uint16_t rx_id = bytes_to_uint16(m->data[0], m->data[1]);
    uint16_t rx_id = (((uint16_t) m->data[0]) << 8) | ((uint16_t) (m->data[1]));
	uint8_t measured_distance_instantaneous = estimate_distance(d);

	for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
		if (neighbor_info_array[i].id == rx_id) {
            // Message from the neighbor is already in table
			can_insert = true;
			new_entry = false;
			index_to_insert = i;
		}
	}
	if (!can_insert) {
		for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
			if (neighbor_info_array[i].id == 0) {
                // There's an empty spot in the table
				can_insert = true;
				new_entry = true;
				index_to_insert = i;
				break;
			}
		}
	}
	if (!can_insert) {
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
            neighbor_info_array[index_to_insert].diffusion_timer = kilo_ticks;
            // Update concentrations based on all 3 belief values sent
            update_concentrations(0, m->data[4]);
            update_concentrations(1, m->data[5]);
            update_concentrations(2, m->data[6]);
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
            if (exp_observation) {
                explore_duration = exp_rand(mean_explore_duration);
            } else {
                explore_duration = mean_explore_duration;
            }
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
                feature_estimate = 255;
                confidence = (double)color_light_dur / (color_light_dur + color_dark_dur);
                //if (confidence > 0) printf("%s LIGHT    (%f, %f)\n", feature_to_color(), confidence, confidence);
            } else if (color_light_dur < color_dark_dur) {
                feature_estimate = 0;
                confidence = (double)color_dark_dur / (color_light_dur + color_dark_dur);
                //if (confidence > 0) printf("%s DARK     (%f, %f)\n", feature_to_color(), 1-confidence, confidence);
            } else {
                feature_estimate = 127;
                confidence = 0;
            }
            detect_feature_state = DETECT_FEATURE_INIT;
            is_feature_disseminating = true;  // Tell message_tx to send updated message
            // Create array of messages to re-disseminate
            if (allow_retransmit) {
                //printf("\nDISSEMINATE:\n");
                //print_neighbor_info_array();
                uint16_t num_neighbors = count_neighbors();
                std::vector<uint32_t> neighbor_inds = mult_rand_int(num_neighbors, num_retransmit);
                retransmit_messages = generate_retransmit_messages(neighbor_inds);

                /*printf("RETRANSMIT:\n");
                for (uint8_t i = 0; i < retransmit_messages.size(); ++i) {
                    printf("%u\t%u\t%u\t%u\t%u\n\r",
                           i,
                           retransmit_messages[i].data[0],
                           retransmit_messages[i].data[1],
                           retransmit_messages[i].data[2],
                           retransmit_messages[i].data[3]);
                }*/

            }
            uint32_t dissemination_duration_mean = dissemination_duration_constant;
            if (use_confidence) {
                dissemination_duration_mean = dissemination_duration_constant * confidence;
            }
            if (exp_dissemination) {
                dissemination_duration = exp_rand(dissemination_duration_mean);
            } else {
                dissemination_duration = dissemination_duration_mean;
            }
            //dissemination_duration = exp_rand(dissemination_duration_constant * confidence);
            dissemination_start_time = kilo_ticks;
        }
    }
}

void update_pattern_beliefs() {
    if (is_updating_belief) {
        is_updating_belief = false;
        //print_neighbor_info_array();
        if (log_debug_info) {
            // Log how many neighbors they used to make decision
            uint16_t num_neighbors = count_neighbors();
            FILE * comm_log = fopen(comm_log_filename.c_str(), "a");
            fprintf(comm_log, "%f\t%d\t%d\t%d\n", float(kilo_ticks)/SECOND, id, detect_which_feature, num_neighbors);
            fclose(comm_log);
        }
        if (pattern_decision_method == DMMD) {
            // Majority-based decisions
            for (uint8_t f = 0; f < NUM_FEATURES; ++f) {
                std::vector<uint8_t> histogram(UINT8_MAX+1, 0);
                for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; ++i) {
                    if (neighbor_info_array[i].id != 0 && neighbor_info_array[i].detect_which_feature == f) {
                        ++histogram[neighbor_info_array[i].feature_estimate];
                    }
                }
                uint8_t new_belief = std::max_element(histogram.begin(), histogram.end()) - histogram.begin();
                if (histogram[new_belief] != 0) {
                    if (histogram[0] == histogram[255]) {
                        // Flip a coin to choose
                        int val = rand() % 2;
                        if (val == 1) {
                            pattern_belief[f] = 0;
                        } else {
                            pattern_belief[f] = 255;
                        }
                    } else {
                        //printf("[%d] %d -> %d\n", f, pattern_belief[f], new_belief);
                        pattern_belief[f] = new_belief;
                    }
                } else {
                    //printf("No update for feature %d\n", f);
                }
                // Else: heard from no one; keep current belief for feature
            }
        } else if (pattern_decision_method == DMVD) {
            // Voter-based decisions (lottery)
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
        // Use pattern belief updates to update concentrations
        /*for (int f = 0; f < 3; f++) {
            update_concentrations(f, pattern_belief[f]);
        }*/
    }
}

void update_concentrations(uint8_t which_feature, uint8_t belief_val) {
    // Update concentration based on concentration in received message
    // TODO: Could also take into account distance of neighbors (delta_concentration/distance)
    // TEMPORARY: Lock beliefs of first and last kilobots (test generating gradient)
    if (!decision_locked[which_feature] && belief_val != 127) {
        float *old_concentrations = concentrations;
        float concentration_change = diffusion_constant * ((float) belief_val / 255 - concentrations[which_feature]);
        //printf("[%d]: %f + %d -> %f\n", which_feature, concentrations[which_feature], belief_val, concentration_change);
        concentrations[which_feature] += concentration_change;
        //if (which_feature == 0 && concentrations[0] < 0.3) {
        //    printf("[update] %u:\t(%f, %f, %f) -> %f -> (%f, %f, %f)\n",
        //           which_feature,
        //           old_concentrations[0], old_concentrations[1], old_concentrations[2],
        //           concentration_change,
        //           concentrations[0], concentrations[1], concentrations[2]);
        //}
    }
}

void decision_checker() {
    // Check if any concentrations are past threshold and start timer(s)
    for (int f = 0; f < 3; f++) {
        if (!decision_locked[f]) {
            if (concentrations[f] < diffusion_decision_thresh || concentrations[f] > 1 - diffusion_decision_thresh) {
                if (diffusion_decision_time < decision_timer[f] + kilo_ticks) {
                    // Has been past threshold long enough to lock decision
                    decision_locked[f] = true;
                    printf("LOCK %d: %d\t(%f, %f, %f)\n",
                           f, id,
                           concentrations[0], concentrations[1], concentrations[2]);
                    if (concentrations[f] < diffusion_decision_thresh) {
                        decision[f] = 0;
                    } else {
                        decision[f] = 255;
                    }
                }
            } else {
                decision_timer[f] = kilo_ticks;
            }
        }
    }
}

// AUXILIARY FUNCTIONS FOR LOOP (DETECTION AND MOVEMENT)

std::vector<uint16_t> sample_light() {
    // FOR SIMULATOR:
    return get_ambientlight();

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
	} else if (rw_state == BOUNCE && curr_light_level != GRAY) {
			// end bounce phase
			rw_state = RW_INIT;
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

    //is_feature_disseminating = true;

	// put your setup code here, to be run only once
	// Set initial light value
    curr_light_level = detect_light_level(detect_which_feature);
	rw_last_changed = kilo_ticks;
    // Give them some color so they're visible
    set_color(RGB(0.5, 0.5, 0.5));
    // Initialize own id
    seed_rng();
	initialize_neighbor_info_array();
	//own_id = rand();
    // TODO: Use ID from robot.h
    /*own_id = id;
	while (own_id == 0 || own_id == 0x1234) {
		own_id = rand();
	}*/
}

void loop() {

    /*if (kilo_ticks == 1) {
        printf("[Initial] %d, (%f, %f, %f)\n", id, concentrations[0], concentrations[1], concentrations[2]);
    }*/

    curr_light_level = detect_light_level(detect_which_feature);

    // Movement
    random_walk(long_rw_mean_straight_dur, rw_max_turn_dur);

    // Feature observation
    detect_feature_color();

    // Neighbor updates
    static uint32_t last_printed;
    neighbor_info_array_locked = true;
    if (new_message) {
        update_neighbor_info_array(&rx_message_buffer, &rx_distance_buffer);
        //print_neighbor_info_array();
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
    // Incorporate own belief update into concentrations


    // Check if decisions can be made from concentrations
    decision_checker();

    // LED: CONCENTRATION
    //set_color(RGB(concentrations[0]/255, concentrations[1]/255, concentrations[2]/255));

    // LED: OWN ESTIMATE
    float c[3] = {concentrations[0], concentrations[1], concentrations[2]};
    for (int f = 0; f < 3; f++) {
        if(decision_locked[f]) {
            if (decision[f] == 255) {
                c[f] = 1;
            } else {
                c[f] = 0;
            }
        }
    }
    set_color(RGB(c[0], c[1], c[2]));

    //set_color(RGB(concentrations[0], concentrations[1], concentrations[2]));

}


// MESSAGE HANDLING

void retransmit_tx_message_data() {
    // Pick a random message from retransmit_messages to re-disseminate & mutate the message to be sent
    // TODO: Possibly update to send specific messages (e.g., oldest, majority value)
    // Currently ignores which feature to re-transmit
    // TODO: Will likely need to worry about neighbor array info locking here?

    if (retransmit_messages.size() > 0) {
        //printf("RETRANSMIT MESSAGE\n");
        uint32_t use_ind = uniform_rand(retransmit_messages.size());
        tx_message_data = retransmit_messages[use_ind];
    }

    // Pick neighbor to retransmit
    /*uint16_t num_neighbors = 0;
    for (uint8_t i = 0; i < NEIGHBOR_INFO_ARRAY_SIZE; i++) {
        if (neighbor_info_array[i].id != 0) {
            num_neighbors++;
        }
    }
    uint32_t neighbor_ind = uniform_rand(num_neighbors);
    neighbor_info_array_t neighbor = neighbor_info_array[neighbor_ind];

    tx_message_data.type = NORMAL;
    // ID
    tx_message_data.data[0] = ((uint8_t) ((neighbor.id & 0xff00) >> 8));
    tx_message_data.data[1] = ((uint8_t) (neighbor.id & 0x00ff));
    // Feature variable measured
    tx_message_data.data[2] = neighbor.detect_which_feature;
    // Estimate of feature
    tx_message_data.data[3] = neighbor.feature_estimate;
    tx_message_data.crc =  message_crc(&tx_message_data);
     */
}

float binary_to_float(void) {
    // Convert the 13-bit binary representation of the number to a float
}

void update_tx_message_data() {
    // TODO: Send message that includes concentration
    tx_message_data.type = NORMAL;
    // ID
    //uint8_t *id_array = uint16_to_bytes(id);
    //tx_message_data.data[0] = id_array[0];
    //tx_message_data.data[1] = id_array[1];
    tx_message_data.data[0] = ((uint8_t) ((id & 0xff00) >> 8));
    tx_message_data.data[1] = ((uint8_t) (id & 0x00ff));
    // Feature varaible measured
    tx_message_data.data[2] = detect_which_feature;
    // Estimate of feature
    tx_message_data.data[3] = feature_estimate;
    // Belief for all 3 features
    for (int f = 0; f < 3; f++) {
        if (decision_locked[f]) {
            tx_message_data.data[f+4] = decision[f];
        } else {
            tx_message_data.data[f+4] = pattern_belief[f];
        }
    }
    //tx_message_data.data[4] = pattern_belief[0];
    //tx_message_data.data[5] = pattern_belief[1];
    //tx_message_data.data[6] = pattern_belief[2];
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
    if (!neighbor_info_array_locked) {
		rx_message_buffer = (*m);
		rx_distance_buffer = (*d);
		new_message = true;
        // TODO: Needs to be moved out of here (to loop() function) for actual kilobots
	}
}

// Executed on successful message send
void message_tx_success() {}

};
