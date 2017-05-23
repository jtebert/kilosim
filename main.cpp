#include "shapes.h"
#pragma warning(disable:4996)

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
#include <vector>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include "vars.h"
//#include "shapes.h"

#define SIMPLEBMP_OPENGL
#include "simplebmp.h"
using namespace std;

#ifndef STAT_INFO
#define STAT_INFO
struct stat info;
#endif

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

robot** robots; //creates an array of robots
int* safe_distance;
int* order;

int delay = delay_init;
int draw_delay = 1;
FILE *results;
FILE *log_file;

std::string log_buffer;
char log_file_buffer[buffer_size];

std::string log_file_name;
bool showscene = true;

char shape_file_name[255] = "";


int total_secs;
int timelimit = 180 * 60;
char rt[100];

double ch[radius];

int snapshot = 60;
int snapshotcounter = 0;

bool last = false;
bool write_final = false;

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

void strcpy_safe(char *m, int l, char *s)
{
	for (int i = 0; i <= l && s; i++)
		*m = *s;
}

void log_str(std::string str) {
	// I don't get what the other log function is doing.
	// This will just save the input string to log_file_name
    if (log_debug_info) {
        log_file = fopen(log_file_name.c_str(), "a");
        fprintf(log_file, "%s", str.c_str());
        fclose(log_file);
    }
}

double convergence_ratio(uint8_t feature) {
	// Find the percentage of kilobots in convergence about belief for given feature
	int count_converge_up = 0;
    int count_converge_down = 1;
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

//check to see if motion causes robots to collide
int find_collisions(int id, double x, double y, double dt)
{
    // Check for collission with wall
	if (x <= radius || x >= arena_width - radius || y <= radius || y >= arena_height - radius) return 2;

	double two_r = 2 * radius;
	int i;
	double x_ulim = x + two_r;
	double x_llim = x - two_r;
	double y_ulim = y + two_r;
	double y_llim = y - two_r;
	for (i = 0;i < num_robots;i++) {
		if (i != id) {
			if (safe_distance[id*num_robots+i]) {
				safe_distance[id*num_robots + i]--;
			} else {
				double dist_x = x - robots[i]->pos[0];
				double dist_y = y - robots[i]->pos[1];
				if (x_ulim > robots[i]->pos[0] && x_llim<robots[i]->pos[0] &&
					y_ulim>robots[i]->pos[1] && y_llim < robots[i]->pos[1]) {
                        //if not in the sqare limits, i dont even check the circular ones
					double distance = sqrt(dist_x*dist_x + dist_y * dist_y);
					if (distance < two_r) {
						return 1;
					}
				} else {
					double bd = 0;
					if (fabs(dist_x)>fabs(dist_y)) {
						bd = fabs(dist_x);
					} else {
						bd = fabs(dist_y);
					}
					if (bd > two_r+20) {
						double speed = (robots[id]->forward_speed + robots[i]->forward_speed) * dt;
						if (speed > 0) {
							safe_distance[id*num_robots + i] = (int)((bd - (two_r + 20)) / speed);
						} else {
							safe_distance[id*num_robots + i] = 1000000;
						}
						safe_distance[i*num_robots + id] = safe_distance[id*num_robots + i];
					}
				}
			}
		}
	}
	return 0;
}

void save_bmp(const char *fileName)
{
	// The width and the height, would be the width
	// and height of your current scene.

	SimpleBMP bmp(windowWidth, windowHeight);

	bmp.glReadPixels();
	bmp.save(fileName);
}

std::vector<double> wall_collision_adjust(double x, double y) {
    // Adjust x, y positions so that it goes to wall if it should be touching
    // Occurs when temporary position is outside of boundaries (put on bound)
    std::vector<double> p = {x, y};
    if (y >= arena_height - radius) {
		p[1] = arena_height - radius;  // Top wall
	} else if (x >= arena_width - radius) {
		p[0] = arena_width - radius;  // Right wall
	} else if ( y <= radius) {
		p[1] = radius;  // Bottom wall
	} else if (x <= radius) {
		p[0] = radius;  // Left wall
	}
    return p;
}

bool run_simulation_step()
{
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
	for (i = 0; i < num_robots; i++) {
		//run controller this time step with p_control_execute probability
		if ((rand())<(int)(p_control_execute*RAND_MAX)) {
			robots[i]->robot_controller();
		}
	}

	int seed;
	seed = (rand() % shuffles) * num_robots;
	//let robots communicate
	for (i = 0; i < num_robots; i++) {
		int index = order[seed + i];
		robot *rs = robots[index];
		//if robot wants to communicate, send message to all robots within distance comm_range
		void *msg = rs->get_message();
		if (msg) {
			for (j = 0; j < num_robots; j++) {
				robot *rd = robots[j];
				if (j != index) {
					double range = rs->comm_out_criteria(rd->pos[0], rd->pos[1], safe_distance[index * num_robots + j]);
					if (range) {
						if (rd->comm_in_criteria(rs->pos[0], rs->pos[1], range, msg)) {
							rs->received();
							//break;
						}
					}
				}
			}
		}
	}

	seed = (rand() % shuffles) * num_robots;
	//move robots
	for (i = 0; i < num_robots; i++) {
		int index = order[seed + i];
		robot *r = robots[index];

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

        int collision_type = find_collisions(index, temp_x, temp_y, dt);
		if (collision_type == 0) {  // No collision
			r->pos[0] = temp_x;
			r->pos[1] = temp_y;
            r->collision_timer = 0;
        } else if (collision_type == 1) {  // Hitting another kilobot
            if (r->collision_turn_dir == 0) {
                theta = r->pos[2] - r->turn_speed * dt;  // left/CCW
            } else {
                theta = r->pos[2] + r->turn_speed * dt;  // right/CW
            }
            if (r->collision_timer > r->max_collision_timer) {  // Change turn dir
                r->collision_turn_dir = (r->collision_turn_dir + 1) % 2;
                r->collision_timer = 0;

            }
            r->collision_timer++;
		}
        // If a bot is touching the wall (collision_type == 2), no position update

        r->pos[2] = wrap_angle(theta);
	}
	static int lastsec =-1;
	bool result = false;

	// Log info at each time step
	std::ostringstream os;
	os << float(lastrun)/SECOND << "\t" << convergence_ratio(0) << "\t" << convergence_ratio(1) << "\t" << convergence_ratio(2) << "\n";
	log_buffer = os.str();

	log_str(log_buffer);

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
    // TODO: use bearing to point in right direction
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

// Drawing routine.
void draw_scene(void)
{
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

        // Draw robots in different shapes depending on feature to detect
        for (int j = 0; j < num_robots; j++) {
            glColor4f((GLfloat)robots[j]->color[0], (GLfloat)robots[j]->color[1], (GLfloat)robots[j]->color[2], 1.0);
            if (robots[j]->detect_which_feature == 0) {
                drawFilledCircle((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1], radius);
            } else if (robots[j]->detect_which_feature == 1) {
                drawFilledTriangle((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1], radius*1.3, robots[j]->pos[2]);
            } else  if (robots[j]->detect_which_feature == 2) {
                drawFilledSquare((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1], radius*1.3, robots[j]->pos[2]);
            }
        }

		glBegin(GL_LINES);
		/*for (int i = 0; i <= radius; i++) {
			for (int j = 0; j < num_robots; j++) {
				glColor4f((GLfloat)robots[j]->color[0], (GLfloat)robots[j]->color[1], (GLfloat)robots[j]->color[2], 1.0);
				glVertex2f((GLfloat)(robots[j]->pos[0]-i), (GLfloat)(robots[j]->pos[1]-ch[i]));
				glVertex2f((GLfloat)(robots[j]->pos[0] -i), (GLfloat)(robots[j]->pos[1] + ch[i]));
				glVertex2f((GLfloat)(robots[j]->pos[0] + i), (GLfloat)(robots[j]->pos[1] - ch[i]));
				glVertex2f((GLfloat)(robots[j]->pos[0] + i), (GLfloat)(robots[j]->pos[1] + ch[i]));
			}
		}*/
		for (int j = 0; j < num_robots; j++) {
			glBegin(GL_LINES);
			glColor4f(0, 0, 0, 1.0);
			glVertex2f((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1]);
			glVertex2f((GLfloat)(robots[j]->pos[0] + cos(robots[j]->pos[2])*radius), (GLfloat)(robots[j]->pos[1] + sin(robots[j]->pos[2])*radius));
			if (robots[j]->dest[0] != -1) {
				glBegin(GL_LINES);
				glColor4f(1, 1, 1, 1.0);
				glVertex2f((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1]);
				glVertex2f((GLfloat)robots[j]->dest[0], (GLfloat)robots[j]->dest[1]);
			}
		}
		glEnd();
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

// Initialization routine.
void setup(void) {
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
	for (int i = 0; i < num_robots; i++)
		for (int j = 0; j < num_robots; j++)
				safe_distance[i * num_robots + j] = 0;
}

// OpenGL window reshape routine.
void resize_window(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, 100.0, 0.0, 100.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// Keyboard input processing routine.
void key_input(unsigned char key, int x, int y) {
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

void setup_positions()
{
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

// Main routine.
int main(int argc, char **argv) {
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
		if (strcmp(argv[i], "--logname") == 0) {
			log_file_name_base = argv[i + 1];
		}
		if (strcmp(argv[i], "--logdir") == 0) {
			log_file_dir = argv[i + 1];
		}
		if (strcmp(argv[i], "--snapshot") == 0) {
			snapshot = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "--seed") == 0) {
			seed = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "--shape") == 0) {
			strcpy_safe(shape_file_name, 255, argv[i + 1]);
		}
		if (strcmp(argv[i], "--trial") == 0) {
			trial_num = stoi(argv[i + 1]);
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
	}

    // Get shapes from file
    std::string shapes_filename = shapes_dir + "/" + shapes_filename_base +
                                  std::to_string(arena_rows) + "x" + std::to_string(arena_rows) +
                                  "-[" + float_to_string(color_fill_ratio[0]) + "," + float_to_string(color_fill_ratio[1]) + "," + float_to_string(color_fill_ratio[2]) + "]-" +
                                  std::to_string(trial_num) + ".txt";
    //polygons = gen_color_squares("shapes/shapes-13x13-[0.3,0.2,0.8]-10.txt");
    polygons = gen_color_squares(shapes_filename);

	// Create directory for logging if it doesn't already exist
	if (stat(log_file_dir.c_str(), &info) != 0) {
		const int dir_err = mkdir(log_file_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		if (-1 == dir_err) {
			printf("Error creating directory!n");
			exit(1);
		}
	}
	log_file_name = log_file_dir + "/" + log_file_name_base + std::to_string(trial_num) + ".log";
	// Check if file exists and warn before overwrite
	if (stat(log_file_name.c_str(), &info) == 0) {
		std::string is_overwrite;
		std::cout << "This log file already exists. Do you want to overwrite it? (y/N) ";
		std::cin >> is_overwrite;
		if (strcmp(is_overwrite.c_str(), "y") == 0) {
			remove(log_file_name.c_str());
		} else {
			std::cout << "File not overwritten. Exiting" << std::endl;
			exit(1);
		}
	}
	std::cout << log_file_name << std::endl;

	robots = (robot **)malloc(num_robots * sizeof(robot *)); //creates an array of robots
	safe_distance = (int *) malloc(num_robots * num_robots * sizeof(int));
	order = (int *) malloc(shuffles * num_robots * sizeof(int));
	//seed random variable for different random behavior every time
	unsigned int t = 0;

	if (seed) {
		t = seed;
	} else {
		t= (unsigned int) time(NULL);
	}

	// Log once at start of simulation
	//sprintf(log_buffer, "random seed: %d\n", t);
	//log_str(log_buffer);

	srand(t);

	//set the simulation time to 0
	time_sim = 0;

	//inital zoom and scroll positions
	zoom = arena_width;
	view_x = arena_width;
	view_y = arena_height;

	//place robots
	//setup_positions_gradient();
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
		while (total_secs<timelimit) {
            char a;
            std::cin >> a;
			run_simulation_step();
		}

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
	}
	return 0;
}
