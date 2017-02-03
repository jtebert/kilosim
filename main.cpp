#pragma warning(disable:4996)

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include "robot.h"
#include "kilobot.cpp"
#include <chrono>
#include <thread>
#include <vector>
//#include "vars.cpp"  // Already included in kilobot.cpp, so including it here breaks

#define SIMPLEBMP_OPENGL
#include "simplebmp.h"
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

int num_robots = 20;  //number of robots running

robot** robots; //creates an array of robots
int* safe_distance;
int* order;

int delay = delay_init;
int draw_delay=1;
FILE *results;

char log_buffer[255];
char log_file_buffer[buffer_size];

bool log_debug_info = true;
char log_file_name[255] = "simulation.log";
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

void log_info(char *s)
{
	static char *m = log_file_buffer;
	if (s)
	{
		int l = strlen(s) + 1;
		strcpy(m, s);
		m += l - 1;
	}
	if (m - log_file_buffer >= buffer_size-255 || !s)
	{
		results = fopen(log_file_name, "a");
		fprintf(results, "%s", log_file_buffer);
		fclose(results);
		m = log_file_buffer;
	}
}
//check to see if motion causes robots to collide
int find_collisions(int id, double x, double y)
{
	double two_r = 2 * radius;
	int i;
	// Check for collission with wall
	if (x <= radius || x >= arena_width - radius || y <= radius || y >= arena_height - radius) return 2;
	double x_ulim = x + two_r;
	double x_llim = x - two_r;
	double y_ulim = y + two_r;
	double y_llim = y - two_r;
	for (i = 0; i < num_robots; i++) {
        // id = this robot;  i = each other robot
		if (i != id) {
			if (safe_distance[id*num_robots+i]) {
				safe_distance[id*num_robots + i]--;
			} else {
                // Distance between x,y positions of this and other robot
				double dist_x = x - robots[i]->pos[0];
				double dist_y = y - robots[i]->pos[1];
				if (x_ulim > robots[i]->pos[0] &&
    				x_llim < robots[i]->pos[0] &&
    				y_ulim > robots[i]->pos[1] && y_llim < robots[i]->pos[1]) {
                    // if not in the sqare limits, i dont even check the circular ones
                    double distance = sqrt(dist_x*dist_x + dist_y * dist_y);
	                if (distance < two_r) {
    					return 1;
    				}
                } else {
                    // bd = biggest distance between 2 robots?
    				double bd = max(fabs(dist_x), fabs(dist_y));
    				/*if (fabs(dist_x) > fabs(dist_y)) {
						bd = fabs(dist_x);
    				} else {
						bd = fabs(dist_y);
    				}*/
                    if (bd > two_r + 20) {
    					double speed = max(robots[id]->forward_speed, robots[id]->turn_speed) + max(robots[id]->forward_speed, robots[id]->turn_speed);
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

	int secs = total_secs % 60;
	int mins = (total_secs / 60) % 60;
	int hours = total_secs / 3600;
	sprintf(rt, "%02d:%02d:%02d", hours, mins, secs);

	int i, j;

	//double rotation_step = .025; //motion step size
    double rotation_step = .05; //motion step size

	//run a step of most or all robot controllers
	for (i = 0; i < num_robots; i++)
	{
		//run controller this time step with p_control_execute probability
		if ((rand())<(int)(p_control_execute*RAND_MAX))
		{
			robots[i]->robot_controller();
		}
	}

	int seed;
	seed = (rand() % shuffles) * num_robots;
	//let robots communicate
	for (i = 0; i < num_robots; i++)
	{
		int index = order[seed + i];
		robot *rs = robots[index];
		//if robot wants to communicate, send message to all robots within distance comm_range
		void *msg = rs->get_message();
		if (msg)
		{
			for (j = 0; j < num_robots; j++)
			{
				robot *rd = robots[j];
				if (j != index)
				{
					double range = rs->comm_out_criteria(rd->pos[0], rd->pos[1], safe_distance[index * num_robots + j]);
					if (range)
					{
						if (rd->comm_in_criteria(rs->pos[0], rs->pos[1], range, msg))
						{
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
	for (i = 0; i < num_robots; i++)
	{
		int index = order[seed + i];
		robot *r = robots[index];

		double theta = r->pos[2];
		double speed = 0;
		switch (r->motor_command) {
    		case 1: {  // forward
    			theta += r->motor_error;
    			speed = r->forward_speed;
    			break;
    		}
    		case 2: {  // CW rotation
    			theta -= rotation_step;
    			speed = r->turn_speed;
    			break;
    		}
    		case 3: { // CCW rotation
    			theta += rotation_step;
    			speed = r->turn_speed;
    			break;
    		}
		}
		double temp_x = speed*cos(theta) + r->pos[0];
		double temp_y = speed*sin(theta) + r->pos[1];
        int collision_type = find_collisions(index, temp_x, temp_y);
		if (collision_type == 0) {  // No collision
			r->pos[0] = temp_x;
			r->pos[1] = temp_y;
            r->collision_timer = 0;
        } else if (collision_type == 1) {  // Hitting another kilobot
            if (r->collision_turn_dir == 0) {
                theta -= rotation_step;  // left/CCW
            } else {
                theta += rotation_step;  // right/CW
            }
            if (r->collision_timer > r->max_collision_timer) {  // Change turn dir
                r->collision_turn_dir = (r->collision_turn_dir + 1) % 2;
                r->collision_timer = 0;

            }
            r->collision_timer++;
		}
        // TODO: If hitting for long time (TBD), switch turn direction of robot
        // If a bot is touching the wall, move it the difference so it is exactly on the edge
        r->pos[2] = wrap_angle(theta);
		//r->pos[2] = theta;
	}
	static int lastsec =-1;
	bool result = false;
/*
    if ((lastsec!=secs && lastrun>1 && snapshot )|| last)
    {
        if (last)
            cout << "ended\n";
        else
            cout << rt << endl;

        lastsec = secs;
        if (!snapshotcounter || last)
        {
            result = true;
            if (log_debug_info || last)
            {
                char finalMSG[] = "final";
                char buffer[255];
                if (last)
                {
                    for (int i = 0;i < num_robots;i++)
                        log_info(robots[i]->get_debug_info(buffer, finalMSG));
                }else
                {
                    for (int i = 0;i < num_robots;i++)
                        log_info(robots[i]->get_debug_info(buffer, rt));
                }
            }
            snapshotcounter = snapshot;
        }
        snapshotcounter--;
    }*/
	if(lastrun%draw_delay==0)
		return true;
	return false;
}

// Drawing routine.
void draw_scene(void)
{
	static int snapshottaken = 0;
	static bool takesnapshot = false;
	//draws the arena

	takesnapshot = run_simulation_step();

	if(takesnapshot)
	{
		glColor4f(0, 0, 0, 0);
		glRectd(0, 0, arena_width, arena_height);

		glutSetWindowTitle(rt);
		glEnable(GL_LINE_SMOOTH);
		glLineWidth(1.0);
		glBegin(GL_LINES);
		for (int i = 0; i <= radius; i++)
		{
			for (int j = 0; j < num_robots; j++)
			{
				glColor4f((GLfloat)robots[j]->color[0], (GLfloat)robots[j]->color[1], (GLfloat)robots[j]->color[2], 1.0);
				glVertex2f((GLfloat)(robots[j]->pos[0]-i), (GLfloat)(robots[j]->pos[1]-ch[i]));
				glVertex2f((GLfloat)(robots[j]->pos[0] -i), (GLfloat)(robots[j]->pos[1] + ch[i]));
				glVertex2f((GLfloat)(robots[j]->pos[0] + i), (GLfloat)(robots[j]->pos[1] - ch[i]));
				glVertex2f((GLfloat)(robots[j]->pos[0] + i), (GLfloat)(robots[j]->pos[1] + ch[i]));
			}
		}
		for (int j = 0; j < num_robots; j++)
		{
			glBegin(GL_LINES);
			glColor4f(0, 0, 0, 1.0);
			glVertex2f((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1]);
			glVertex2f((GLfloat)(robots[j]->pos[0] + cos(robots[j]->pos[2])*radius), (GLfloat)(robots[j]->pos[1] + sin(robots[j]->pos[2])*radius));
			if (robots[j]->dest[0] != -1)
			{
				glBegin(GL_LINES);
				glColor4f(1, 1, 1, 1.0);
				glVertex2f((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1]);
				glVertex2f((GLfloat)robots[j]->dest[0], (GLfloat)robots[j]->dest[1]);
			}
		}
		glEnd();
		glFlush();

		/*if (takesnapshot)
		   {
		    snapshottaken++;
		    char file[100];
		    if (last)
		    {
		        sprintf(file, "%s.final.bmp", log_file_name);
		    }
		    else
		    {
		        sprintf(file, "%s.%04d.bmp", log_file_name, snapshottaken);
		    }
		    save_bmp(file);
		   }*/

		glutSwapBuffers();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	}

	if (last)
	{
		log_info(NULL);
		exit(0);
	}
	if (total_secs >= timelimit)
	{
		last = true;
	}
}

// Initialization routine.
void setup(void)
{
	for (int i = 0; i < num_robots; i++)
		for (int j = 0; j < shuffles; j++)
				order[i + num_robots*j] = i;

	for (int i = 0; i < num_robots - 1; i++)
		for (int j = 0; j < shuffles; j++)
		{
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
void key_input(unsigned char key, int x, int y)
{
	switch (key)
	{
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
	for (int i = 0; i < num_robots; i++)
	{
		int c = i % columns + 1;
		int r = i / columns + 1;
		int hr = rand() % (horizontal_separation / 2) + horizontal_separation / 4;
		int x = c * horizontal_separation; // + hr;
		int vr = rand() % (vertical_separation / 2) + vertical_separation / 4;
		int y = r * vertical_separation; // + vr;
		robots[k] = new mykilobot();
		double theta = rand() * 2 * PI / RAND_MAX;
		robots[k]->robot_init(x, y, theta);
        if (k == 0) {
            robots[k]->set_color(RGB(0,0,1));
        }
        track_id = robots[k]->id;
		k++;
	}
}

// Main routine.
int main(int argc, char **argv)
{
	for (int i = 0; i < argc-1; i++)
	{
		if (strcmp(argv[i],"/r")==0)
		{
			num_robots = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/l") == 0)
		{
			log_debug_info = argv[i + 1][0]=='y';
		}
		if (strcmp(argv[i], "/d") == 0)
		{
			showscene = argv[i + 1][0] == 'y';
		}
		if (strcmp(argv[i], "/aw") == 0)
		{
			arena_width = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/ah") == 0)
		{
			arena_height = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/t") == 0)
		{
			timelimit = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/f") == 0)
		{
			strcpy_safe(log_file_name,255, argv[i + 1]);
		}
		if (strcmp(argv[i], "/ss") == 0)
		{
			snapshot = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/seed") == 0)
		{
			seed = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/shape") == 0)
		{
			strcpy_safe(shape_file_name, 255, argv[i + 1]);
		}
	}

	robots = (robot **)malloc(num_robots * sizeof(robot *)); //creates an array of robots
	safe_distance = (int *) malloc(num_robots * num_robots * sizeof(int));
	order = (int *) malloc(shuffles * num_robots * sizeof(int));
	//seed random variable for different random behavior every time
	unsigned int t = 0;

	if (seed)
	{
		t = seed;
	}
	else
	{
		t= (unsigned int) time(NULL);
	}

	sprintf(log_buffer, "random seed: %d\n", t);

	log_info(log_buffer);
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

	for (int i = 0; i < radius; i++)
	{
		ch[i] = sqrt(radius*radius - i*i);
	}

	if (showscene)
	{
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
	else {
		while (total_secs<timelimit)
		{
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
