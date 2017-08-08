#include "shapes.h"
#pragma warning(disable:4996)

#include <omp.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include "robot.h"
#include "kilobot.cpp"
#include <chrono>
#include <thread>
#include <sys/stat.h>
#include "vars.h"

using namespace std;

uint track_id;

rgb RGB(double r, double g, double b)
{
    rgb c;
    c.red = r;
    c.green = g;
    c.blue = b;
    return c;
}

// Global vars.
double time_sim;  //simulation time
double zoom, view_x, view_y; //var. for zoom and scroll

double decide0[2] = {0,0};
double decide1[2] = {0,0};
double decide2[2] = {0,0};

robot** robots; //creates an array of robots
int* order;

int delay = delay_init;
int draw_delay = 1;
FILE *log_file;
std::string log_filename;
std::string decision_filename;
std::string comm_log_filename;
std::string params_filename;

std::string log_buffer;

char shape_file_name[255] = "";


int total_secs;
char rt[100];

double ch[radius];

bool last = false;

unsigned int seed = 0;

double wrap_angle(double angle) {
    // Guarantee that angle will be from 0 to 2*pi
    // While loop is fastest option when angles are close to correct range
    while (angle > TWO_PI) {
        angle -= TWO_PI;
    }
    while (angle < 0) {
        angle += TWO_PI;
    }
    return angle;
}

void parse_use_features(std::string str) {
    // Parse the input string into new values for use_features
    use_features = {};
    std::string delimiter = "-";
    size_t pos = 0;
    std::string token;
    while((pos = str.find(delimiter)) != std::string::npos) {
        token = str.substr(0, pos);
        use_features.push_back((uint8_t)stoi(token.c_str()));
        str.erase(0, pos + delimiter.length());
    }
    use_features.push_back((uint8_t)stoi(str.c_str()));
}

bool use_features_valid() {
    // Check that all the features are valid
    if (use_features.size() == 0) {
        return false;
    }
    for (int i; i < use_features.size(); i++) {
        if (use_features[i] >= num_features) {
            return false;
        }
    }
    return true;
}

void log_str(std::string filename, std::string str) {
	// I don't get what the other log function is doing.
	// This will just save the input string to the filename
    if (log_debug_info) {
        log_file = fopen(filename.c_str(), "a");
        fprintf(log_file, "%s", str.c_str());
        fclose(log_file);
    }
}

double convergence_ratio(uint8_t feature) {
	// Find the percentage of kilobots in convergence about belief for given feature
	int count_converge_up = 0;
    int count_converge_down = 0;
    //#pragma parallel for
	for (int i = 0; i < num_robots; i++) {
        if (robots[i]->pattern_belief[feature] > 127) {
            count_converge_up++;
        } else if (robots[i]->pattern_belief[feature] < 127) {
            count_converge_down++;
        }
    }
	double convergence = double(max(count_converge_up, count_converge_down)) / num_robots;
	return convergence;
}

double *decision_ratio(uint8_t feature, double *decision_rate) {
    // Find the percentage of kilobots that have DECIDED on the feature
    int count_decide_up = 0;
    int count_decide_down = 0;
    for (int i = 0; i < num_robots; i++) {
        if (robots[i]->decision[feature] == 255) {
            count_decide_up += 1;
        } else if (robots[i]->decision[feature] == 0) {
            count_decide_down += 1;
        }
    }
    decision_rate[0] = (double)count_decide_down / num_robots;
    decision_rate[1] = (double)count_decide_up / num_robots;
    return decision_rate;
}

double mean_belief(uint8_t feature) {
    // Get the mean belief across features
    int sum_belief = 0;
    for (int i = 0; i < num_robots; i++) {
        sum_belief += robots[i]->pattern_belief[feature];
    }
    double avg = (double)sum_belief/255/num_robots;
    return avg;
}

double* compute_next_step (double dt) {
	// Compute the next positions of the robots according to their current positions and motor commands
	// This will be used for collision detection (check if they'd be overlapping on the next step)
	// Does NOT address turning that happens when there is a collision
	// new positions are serialized: [x_1, y_1, theta_1, x_2, y_2, theta_2, x_3, ...]

	double *new_pos = (double*)malloc( num_robots * 3 * sizeof(double) );

    #pragma omp parallel for
	for (int i = 0; i < num_robots; i++) {
		robot *r = robots[i];

		double theta = r->pos[2];
		double x = r->pos[0];
		double y = r->pos[1];
		double temp_x = x;;
		double temp_y = y;
		double temp_cos, temp_sin, phi;
		switch (r->motor_command) {
			case 1: {  // forward
				//theta += r->motor_error * dt;
				double speed = r->forward_speed * dt;
				temp_x = speed*cos(theta) + r->pos[0];
				temp_y = speed*sin(theta) + r->pos[1];
				break;
			}
			case 2: {  // CW rotation
				phi = -r->turn_speed * dt;
				theta += phi;
				temp_cos = radius * cos(theta + 4*PI/3);
				temp_sin = radius * sin(theta + 4*PI/3);
				temp_x = x + temp_cos - temp_cos*cos(phi) + temp_sin*sin(phi);
				temp_y = y + temp_sin - temp_cos*sin(phi) - temp_sin*cos(phi);
				break;
			}
			case 3: { // CCW rotation
				phi = r->turn_speed * dt;
				theta += phi;
				temp_cos = radius * cos(theta + 2*PI/3);
				temp_sin = radius * sin(theta + 2*PI/3);
				temp_x = x + temp_cos - temp_cos*cos(phi) + temp_sin*sin(phi);
				temp_y = y + temp_sin - temp_cos*sin(phi) - temp_sin*cos(phi);
				break;
			}
		}

		new_pos[i*3] = temp_x;
		new_pos[i*3 + 1] = temp_y;
		new_pos[i*3 + 2] = wrap_angle(theta);
	}
	return new_pos;
}

int find_collisions(double* new_pos, int self_id, int time) {
	// Check to see if motion causes robots to collide with their updated positions
	double self_x = new_pos[self_id*3];
	double self_y = new_pos[self_id*3 + 1];

	// Check for collision with wall
	if (self_x <= radius || self_x >= arena_width - radius || self_y <= radius || self_y >= arena_height - radius) {
		return 2;
	}
    bool abort = false;

    #pragma omp parallel for
	for (int other_id = 0; other_id < num_robots; other_id++) {
        #pragma omp flush (abort)
        if (!abort) {
            if (other_id != self_id) {  // Don't compare to self
                double other_x = new_pos[other_id * 3];
                double other_y = new_pos[other_id * 3 + 1];
                // Get distance to other robots
                double dist_x = self_x - other_x;
                double dist_y = self_y - other_y;
                double distance = sqrt(pow(dist_x, 2) + pow(dist_y, 2));

                // Check if new positions are intersecting
                if (distance < 2 * radius) {
                    abort = true;
                    #pragma omp flush (abort)
                }
            }
        }
	}
    if (abort) {
        return 1;
    } else {
        // Otherwise no collisions
        return 0;
    }
}

bool run_simulation_step() {
	static int lastrun = 0;
	lastrun++;

	total_secs = lastrun / SECOND;
    double dt = 1.0 / SECOND;  // Time change for speed determination

	int secs = total_secs % 60;
	int mins = (total_secs / 60) % 60;
	int hours = total_secs / 3600;
	sprintf(rt, "%02d:%02d:%02d", hours, mins, secs);

	int i, j;

	//double rotation_step = .025; //motion step size
    double rotation_step = .05; //motion step size

	//run a step of most or all robot controllers
    #pragma omp parallel for
	for (i = 0; i < num_robots; i++) {
        //printf("%d\n", i);
		//run controller this time step with p_control_execute probability
		if ((rand())<(int)(p_control_execute*RAND_MAX)) {
			robots[i]->robot_controller();
		}
	}

	// COMMUNICATION
	// Only communicate at tick rate achievable by kilobots (simulate CSMA/CD)
	if (lastrun % comm_rate == 0) {
        seed = (rand() % shuffles) * num_robots;
        #pragma omp parallel for
		for (int t = 0; t < num_robots; t++) {
			// Loop over all transmitting robots
            int tx_id = order[seed + t];
			robot *tx_r = robots[tx_id];
			void *msg = tx_r->get_message();
			if (msg) {
				for (int rx_id = 0; rx_id < num_robots; rx_id++) {
					// Loop over receivers if transmitting robot is sending a message
					robot *rx_r = robots[rx_id];
					if (tx_id != rx_id) {
						// Check communication range in both directions (due to potentially noisy communication range)
						double dist = tx_r->distance(tx_r->pos[0], tx_r->pos[1], rx_r->pos[0], rx_r->pos[1]);
                        if (tx_r->comm_out_criteria(dist) && rx_r->comm_in_criteria(dist, msg)) {
                            rx_r->received();
                        }
					}
				}
			}
		}
	}

	// MOVEMENT
	double* new_pos = compute_next_step(dt);
	for (int r_id = 0; r_id < num_robots; r_id++) {
		robot *r = robots[r_id];
		double new_x = new_pos[r_id*3];
		double new_y = new_pos[r_id*3 + 1];
		double new_theta = new_pos[r_id*3 + 2];

		int collision_type = find_collisions(new_pos, r_id, lastrun);

		if (collision_type == 0) {  // No collision
			r->pos[0] = new_x;
			r->pos[1] = new_y;
			r->collision_timer = 0;
		} else if (collision_type == 1) {  // Hitting another kilobot
			if (r->collision_turn_dir == 0) {
				new_theta = r->pos[2] - r->turn_speed * dt;  // left/CCW
			} else {
				new_theta = r->pos[2] + r->turn_speed * dt;  // right/CW
			}
			if (r->collision_timer > r->max_collision_timer) {  // Change turn dir
				r->collision_turn_dir = (r->collision_turn_dir + 1) % 2;
				r->collision_timer = 0;

			}
			r->collision_timer++;
		}
        r->pos[2] = wrap_angle(new_theta);
		// If a bot is touching the wall (collision_type == 2), update angle but not position
	}


	static int lastsec =-1;
	bool result = false;

	// Save convergence data
	std::ostringstream os;
	os << float(lastrun)/SECOND << "\t"
       << convergence_ratio(0) << "\t" << convergence_ratio(1) << "\t" << convergence_ratio(2)
       << mean_belief(0) << "\t" << mean_belief(1) << "\t" << "\t" << mean_belief(2) << "\n";
	log_buffer = os.str();
	log_str(log_filename, log_buffer);

    // Save decision-making data
    std::ostringstream os1;
    decision_ratio(0, decide0);
    decision_ratio(1, decide1);
    decision_ratio(2, decide2);
    os1 << (float)lastrun/SECOND << "\t" << decide0[0] << "\t" << decide0[1] << "\t"
        << decide1[0] << "\t" << decide1[1] << "\t"
        << decide2[0] << "\t" << decide2[1] << "\t" << "\n";
    log_buffer = os1.str();
    log_str(decision_filename, log_buffer);


    if (lastrun % (30 * SECOND) == 0) {
        printf("\n[%.1f min]\n", (float)lastrun/SECOND/60);
        printf("DECIDE DOWN: (%f, %f, %f)\n", decide0[0], decide1[0], decide2[0]);
        printf("DECIDE UP:   (%f, %f, %f)\n", decide0[1], decide1[1], decide2[1]);
        printf("MEAN BELIEF: (%f, %f, %f)\n", mean_belief(0), mean_belief(1), mean_belief(2));
    }

    return lastrun % draw_delay == 0;
}

void drawFilledCircle(GLfloat x, GLfloat y, GLfloat radius) {
	int i;
	int triangleAmount = 100; //# of triangles used to draw circle

	//GLfloat radius = 0.8f; //radius
	GLfloat twicePi = 2.0f * PI;

	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(x, y); // center of circle
	for(i = 0; i <= triangleAmount;i++) {
		glVertex2f(
            x + (radius * cos(i *  twicePi / triangleAmount)),
		    y + (radius * sin(i * twicePi / triangleAmount))
		);
	}
	glEnd();
}

void drawFilledTriangle(GLfloat x, GLfloat y, double rad, double theta) {
    // Draw a filled triangle at the specified location with bearing theta
    // size = equilateral triangle of size inscribed by circle with given radius
    glBegin(GL_TRIANGLES);
    glVertex2f(x+rad*cos(theta), y+rad*sin(theta));
    glVertex2f(x+rad*cos(theta+2*PI/3), y+rad*sin(theta+2*PI/3));
    glVertex2f(x+rad*cos(theta+4*PI/3), y+rad*sin(theta+4*PI/3));
    glEnd();
}

void drawFilledSquare(GLfloat x, GLfloat y, double rad, double theta) {
    // Draw a rectange at the specified location with bearing theta
    // size = square of size inscribed in circle with given radius
    glBegin(GL_QUADS);
    glVertex2f(x+rad*cos(theta-PI/4), y+rad*sin(theta-PI/4));
    glVertex2f(x+rad*cos(theta+PI/4), y+rad*sin(theta+PI/4));
    glVertex2f(x+rad*cos(theta+3*PI/4), y+rad*sin(theta+3*PI/4));
    glVertex2f(x+rad*cos(theta+5*PI/4), y+rad*sin(theta+5*PI/4));
    glEnd();
}

void draw_scene(void) {
    // Drawing routine
	static int snapshottaken = 0;
	static bool takesnapshot = false;
	//draws the arena

	takesnapshot = run_simulation_step();

	if(takesnapshot) {
        //Background
		glColor3f(.15, .15, .15);
		glRectd(0, 0, arena_width, arena_height);

        // Draw projected shapes (background)
        for (int i = 0; i < polygons.size(); i++) {
            std::vector<float> c = polygons[i].color;
            glColor3f(c[0]*.6, c[1]*.6, c[2]*.6);
            glBegin(GL_POLYGON);
            for (int j = 0; j < polygons[i].points.size(); j++) {
                glVertex2f(polygons[i].points[j].x, polygons[i].points[j].y);
            }
            glEnd();
        }
        for (int i = 0; i < rects.size(); i++) {
            std::vector<float> c = rects[i].color;
            glColor3f(c[0]*.6, c[1]*.6, c[2]*.6);
			glRectd(rects[i].pos.x, rects[i].pos.y, rects[i].pos.x + rects[i].width, rects[i].pos.y + rects[i].height);
        }
        glColor3f(.6, .6, .6);
        for (int i = 0; i < circles.size(); i++) {
            drawFilledCircle(circles[i].x, circles[i].y, circles[i].rad);
        }

        // Borderlands/edge
        glColor3f(.3, .3, .3);
        glRectd(0, 0, edge_width, arena_height);  // left
        glRectd(0, 0, arena_width, edge_width);  // Bottom
        glRectd(arena_width-edge_width, 0, arena_width, arena_height);  // right
        glRectd(0, arena_height, arena_width, arena_height-edge_width);  // Top

		glutSetWindowTitle(rt);
		glEnable(GL_LINE_SMOOTH);
		glLineWidth(1.0);

        for (int j = 0; j < num_robots; j++) {
            // Draw robots in different shapes depending on feature to detect
            glColor4f((GLfloat)robots[j]->color[0], (GLfloat)robots[j]->color[1], (GLfloat)robots[j]->color[2], 1.0);
            if (robots[j]->detect_which_feature == 0) {
                drawFilledCircle((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1], radius);
            } else if (robots[j]->detect_which_feature == 1) {
                drawFilledTriangle((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1], radius*1.3, robots[j]->pos[2]);
            } else  if (robots[j]->detect_which_feature == 2) {
                drawFilledSquare((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1], radius*1.3, robots[j]->pos[2]);
            }
            // Draw lines for bearing
            glBegin(GL_LINES);
            glColor4f(0.2, 0.2, 0.2, 1.0);
            glVertex2f((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1]);
            glVertex2f((GLfloat)(robots[j]->pos[0] + cos(robots[j]->pos[2])*radius), (GLfloat)(robots[j]->pos[1] + sin(robots[j]->pos[2])*radius));
            glEnd();
        }

		glFlush();

		glutSwapBuffers();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	if (last) {
		//log_info(NULL);
		exit(0);
	}
	if (total_secs >= timelimit) {
		last = true;
	}
}

void setup(void) {
    // Initialization routine.
	for (int i = 0; i < num_robots; i++)
		for (int j = 0; j < shuffles; j++)
				order[i + num_robots*j] = i;

	for (int i = 0; i < num_robots - 1; i++)
		for (int j = 0; j < shuffles; j++) {
			int index = j*num_robots + i;
			int r = index + rand() % (num_robots - i);
			int p = order[index];
			order[index] = order[r];
			order[r] = p;
		}
}

void resize_window(int w, int h) {
    // OpenGL window reshape routine.
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, 100.0, 0.0, 100.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void key_input(unsigned char key, int x, int y) {
    // Keyboard input processing routine.
	switch (key) {
	case 27:
		exit(0);
		break;
	case 'w': //up
		view_y += 100;
		break;
	case 'a': //up
		view_x -= 100;
		break;
	case 's': //up
		view_y -= 100;
		break;
	case 'd': //up
		view_x += 100;
		break;
	case '-':
		zoom = zoom*1.1;
		break;
	case '+':
		zoom = zoom*0.9;
		break;
	case '1':
		if (delay>0)
				delay--;
		printf("busy delay %d\n\r",delay);
		break;
	case '2':
		delay++;
		printf("busy delay %d\n\r",delay);
		break;
	case '3':
		if (draw_delay>1)
				draw_delay--;
		printf("draw delay %d\n\r",draw_delay);
		break;
    	case '4':
			draw_delay++;
			printf("draw delay %d\n\r",draw_delay);
			break;
	default:
		break;
	}
}

void on_idle(void) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-zoom + view_x, view_x, -zoom + view_y, view_y, 0.0f, 1.0f);
	std::this_thread::sleep_for(std::chrono::milliseconds(delay));
	glutPostRedisplay();
}

void setup_positions() {
	int k = 0;
	int columns = (int)sqrt((num_robots * arena_width / arena_height));
	int rows = (int)(num_robots / columns);
	if (num_robots % columns) rows++;
	int horizontal_separation = arena_width / (columns + 1);
	int vertical_separation = (int)arena_height / (rows + 1);
	for (int i = 0; i < num_robots; i++) {
		int c = i % columns + 1;
		int r = i / columns + 1;
		int hr = rand() % (horizontal_separation / 2) + horizontal_separation / 4;
		int x = c * horizontal_separation; // + hr;
		int vr = rand() % (vertical_separation / 2) + vertical_separation / 4;
		int y = r * vertical_separation; // + vr;
		robots[k] = new mykilobot();
		double theta = rand() * 2 * PI / RAND_MAX;
		robots[k]->robot_init(x, y, theta);
        track_id = robots[k]->id;
		k++;
	}
}

void parse_params(int argc, char **argv) {
    // Parse input parameters and mutate appropriate global variables
    for (int i = 0; i < argc-1; i++) {
        if (strcmp(argv[i],"--robots")==0) {
            num_robots = stoi(argv[i + 1]);
        }
        if (strcmp(argv[i], "--log") == 0) {
            log_debug_info = argv[i + 1][0]=='y';
        }
        if (strcmp(argv[i], "--draw") == 0) {
            showscene = argv[i + 1][0] == 'y';
        }
        if (strcmp(argv[i], "--width") == 0) {
            arena_width = stoi(argv[i + 1]);
        }
        if (strcmp(argv[i], "--height") == 0) {
            arena_height = stoi(argv[i + 1]);
        }
        if (strcmp(argv[i], "--time") == 0) {
            timelimit = stoi(argv[i + 1]);
        }
        if (strcmp(argv[i], "--logdir") == 0) {
            log_file_dir = argv[i + 1];
        }
        if (strcmp(argv[i], "--logname") == 0) {
            log_filename_base = argv[i + 1];
        }
        if(strcmp(argv[i], "--decision_logname") == 0) {
            decision_filename = argv[i + 1];
        }
        if (strcmp(argv[i], "--dissemination_dur") == 0) {
            dissemination_duration_constant = (uint32_t)(stoi(argv[i + 1]) * SECOND);
        }
        if (strcmp(argv[i], "--observation_dur") == 0) {
            mean_explore_duration = (uint32_t)(stoi(argv[i + 1]) * SECOND);
        }
        if (strcmp(argv[i], "--seed") == 0) {
            seed = (uint)stoi(argv[i + 1]);
        }
        if (strcmp(argv[i], "--shape") == 0) {
            shapes_filename_base = argv[i + 1];
        }
        if (strcmp(argv[i], "--trial") == 0) {
            trial_num = stoi(argv[i + 1]);
        }
        if (strcmp(argv[i], "--features") == 0) {
            parse_use_features(argv[i + 1]);
        }
        if (strcmp(argv[i], "--rows") == 0) {
            arena_rows = stoi(argv[i + 1]);
        }
        if (strcmp(argv[i], "-r") == 0) {
            color_fill_ratio[0] = stof(argv[i + 1]);
        }
        if (strcmp(argv[i], "-g") == 0) {
            color_fill_ratio[1] = stof(argv[i + 1]);
        }
        if (strcmp(argv[i], "-b") == 0) {
            color_fill_ratio[2] = stof(argv[i + 1]);
        }
        if (strcmp(argv[i], "--use_confidence") == 0) {
            use_confidence = argv[i + 1][0] == 'y';
        }
        if (strcmp(argv[i], "--allow_retransmit") == 0) {
            allow_retransmit = argv[i + 1][0] == 'y';
        }
        /*if (strcmp(argv[i], "--num_retransmit") == 0) {
            num_retransmit = (uint32_t) stoi(argv[i + 1]);
        }*/
        if (strcmp(argv[i], "--comm_rate") == 0) {
            comm_rate = (uint8_t) stoi(argv[i + 1]);
        }
        if (strcmp(argv[i], "--exp_observation") == 0) {
            exp_observation = argv[i + 1][0] == 'y';
        }
        if (strcmp(argv[i], "--exp_dissemination") == 0) {
            exp_dissemination = argv[i + 1][0] == 'y';
        }
        if (strcmp(argv[i], "--comm_dist") == 0) {
            comm_dist = stof(argv[i + 1]);
        }
        if (strcmp(argv[i], "--neighbor_dur") == 0) {
            neighbor_info_array_timeout = (uint32_t)stoi(argv[i + 1]) * SECOND;
        }
        if (strcmp(argv[i], "--num_threads") == 0) {
            num_threads = stoi(argv[i + 1]);
        }
        // Diffusion parameters
        if (strcmp(argv[i], "--diffusion_constant") == 0) {
            diffusion_constant = stof(argv[i + 1]);
        }
        if (strcmp(argv[i], "--diffusion_decision_thresh") == 0) {
            diffusion_decision_thresh= stof(argv[i + 1]);
        }
        if (strcmp(argv[i], "--diffusion_decision_time") == 0) {
            diffusion_decision_time = (uint32_t)stoi(argv[i + 1]);
        }
    }
}

void save_params() {
    std::string params_header, params_vals;
    params_header += "num_robots\t"; params_vals += std::to_string(num_robots) + "\t";
    params_header += "dissemination_dur\t"; params_vals += std::to_string(dissemination_duration_constant/SECOND) + "\t";
    params_header += "observation_dur\t"; params_vals += std::to_string(mean_explore_duration/SECOND) + "\t";
    params_header += "num_rows\t"; params_vals += std::to_string(arena_rows) + "\t";
    params_header += "fill_0\t"; params_vals += std::to_string(color_fill_ratio[0]) + "\t";
    params_header += "fill_1\t"; params_vals += std::to_string(color_fill_ratio[1]) + "\t";
    params_header += "fill_2\t"; params_vals += std::to_string(color_fill_ratio[2]) + "\t";
    params_header += "comm_range\t"; params_vals += std::to_string(comm_dist) + "\t";
    params_header += "exp_observation\t"; params_vals += std::to_string(exp_observation) + "\t";
    params_header += "exp_dissemination\t"; params_vals += std::to_string(exp_dissemination) + "\t";
    params_header += "use_confidence\t"; params_vals += std::to_string(use_confidence) + "\t";
    params_header += "allow_retransmit\t"; params_vals += std::to_string(allow_retransmit) + "\t";
    //params_header += "num_retransmit\t"; params_vals += std::to_string(num_retransmit) + "\t";
    params_header += "comm_rate\t"; params_vals += std::to_string(comm_rate) + "\t";
    params_header += "neighbor_dur\t"; params_vals += std::to_string(neighbor_info_array_timeout/SECOND) + "\t";
    params_header += "diffusion_constant\t"; params_vals += std::to_string(diffusion_constant) + "\t";
    params_header += "diffusion_decision_thresh\t"; params_vals += std::to_string(diffusion_decision_thresh) + "\t";
    params_header += "diffusion_decision_time\t"; params_vals += std::to_string((float)diffusion_decision_time/SECOND) + "\t";
    FILE * params_log = fopen(params_filename.c_str(), "a");
    fprintf(params_log, "%s\n", params_header.c_str());
    fprintf(params_log, "%s\n", params_vals.c_str());
    fclose(params_log);
}

int main(int argc, char **argv) {

    // Main routine.
	parse_params(argc, argv);

    // OpenMP settings
    if (num_threads != 0) {
        omp_set_dynamic(0);     // Explicitly disable dynamic teams
        omp_set_num_threads(num_threads); // Use num_threads for all consecutive parallel regions
    }

    // Check feature values
    if (!use_features_valid()) {
        printf("ERROR: All features IDs used must be 0-%d\n", num_features);
        exit(1);
    }
    /*if (allow_retransmit && num_retransmit <= 0) {
        printf("ERROR: num_retransmit must be a positive integer (not 0)\n");
        exit(1);
    }*/

    // Get shapes from file
    std::string shapes_filename = shapes_dir + "/" + shapes_filename_base +
                                  std::to_string(arena_rows) + "x" + std::to_string(arena_rows) +
                                  "-[" + float_to_string(color_fill_ratio[0]) + "," + float_to_string(color_fill_ratio[1]) + "," + float_to_string(color_fill_ratio[2]) + "]-" +
                                  std::to_string(trial_num) + ".txt";
    //polygons = gen_color_polys(shapes_filename);
    rects = gen_color_rects(shapes_filename);

	// Create directory for logging if it doesn't already exist
	struct stat info;
	if (stat(log_file_dir.c_str(), &info) != 0) {
		const int dir_err = mkdir(log_file_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		if (-1 == dir_err) {
			printf("Error creating directory!\n");
			exit(1);
		}
	}
    log_filename = log_file_dir + "/" + log_filename_base + std::to_string(trial_num) + ".log";
    decision_filename = log_file_dir + "/" + decision_filename_base + std::to_string(trial_num) + ".log";
    comm_log_filename = log_file_dir + "/" + comm_filename_base + std::to_string(trial_num) + ".log";
    params_filename = log_file_dir + "/" + params_filename_base + std::to_string(trial_num) + ".log";
    std::cout << log_filename << std::endl;
	// Check if file exists and warn before overwrite
	if (stat(log_filename.c_str(), &info) == 0) {
		std::string is_overwrite;
		std::cout << "This log file already exists. Do you want to overwrite it? (y/N) ";
		std::cin >> is_overwrite;
		if (strcmp(is_overwrite.c_str(), "y") == 0) {
			remove(log_filename.c_str());
            remove(decision_filename.c_str());
            remove(comm_log_filename.c_str());
            remove(params_filename.c_str());
		} else {
			std::cout << "File not overwritten. Exiting" << std::endl;
			exit(1);
		}
	}

    // DEBUGGING
    // Print stuff here to test parameters/variables

	robots = (robot **)malloc(num_robots * sizeof(robot *)); //creates an array of robots
	order = (int *) malloc(shuffles * num_robots * sizeof(int));
	//seed random variable for different random behavior every time
	unsigned int t = 0;

	if (seed) {
		t = seed;
	} else {
		t= (unsigned int) time(NULL);
	}

	// Save parameters to file
	save_params();

	// LOG ONCE AT START OF SIMULATION
    // Put header line into decisions log file
    std::string decision_header = "time\tdecide_0_down\tdecide_0_up\tdecide_1_down\tdecide_1_up\tdecide_2_down\tdecide_2_up";
    FILE * decision_log = fopen(decision_filename.c_str(), "a");
    fprintf(decision_log, "%s\n", decision_header.c_str());
    fclose(decision_log);
    // Header for default log file
    std::string log_header = "time\tconverge_0\tconverge_1\tconverge_2\tmean_belief_0\tmean_belief_1\tmean_belief_2";
    FILE * log = fopen(log_filename.c_str(), "a");
    fprintf(log, "%s\n", log_header.c_str());
    fclose(log);

	srand(t);

	//set the simulation time to 0
	time_sim = 0;

	//inital zoom and scroll positions
	zoom = arena_width;
	view_x = arena_width;
	view_y = arena_height;

    // Place robots
    setup_positions();

	setup();

	//do some open gl stuff

	for (int i = 0; i < radius; i++) {
		ch[i] = sqrt(radius*radius - i*i);
	}

	if (showscene) {
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
		glutInitWindowSize(windowWidth, windowHeight);
		glutInitWindowPosition(0, 0);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0.0f, 1000, 1000, 0.0f, 0.0f, 1.0f);
		glClearColor(1.0, 1.0, 1.0, 0.0);
		glutCreateWindow("Kilobot simulator");

		glutDisplayFunc(draw_scene);
		glutReshapeFunc(resize_window);
		glutIdleFunc(on_idle);
		glutKeyboardFunc(key_input);
		glutMainLoop();
	} else {
		while (total_secs < timelimit) {
			run_simulation_step();
		}
	}
	return 0;
}
