#include "shapes.h"
#pragma warning(disable : 4996)

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
double time_sim;             //simulation time
double zoom, view_x, view_y; //var. for zoom and scroll

double decide0[2] = {0, 0};
double decide1[2] = {0, 0};
double decide2[2] = {0, 0};

std::vector<Robot *> robots;
int *order;

int delay = delay_init;
int draw_delay = 1;
FILE *log_file;
std::string log_filename;
std::string decision_filename;
std::string comm_log_filename;
std::string params_filename;

std::string log_buffer;

int total_secs;
char rt[100];

double ch[radius];

bool last = false;

unsigned int seed = 0;

double wrap_angle(double angle)
{
    // Guarantee that angle will be from 0 to 2*pi
    // While loop is fastest option when angles are close to correct range
    while (angle > TWO_PI)
    {
        angle -= TWO_PI;
    }
    while (angle < 0)
    {
        angle += TWO_PI;
    }
    return angle;
}

void setup_positions()
{
    int k = 0;
    int columns = (int)sqrt((num_robots * arena_width / arena_height));
    int rows = (int)(num_robots / columns);
    if (num_robots % columns)
        rows++;
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
        robots[k] = new MyKilobot();
        double theta = rand() * 2 * PI / RAND_MAX;
        robots[k]->robot_init(x, y, theta);
        track_id = robots[k]->id;
        k++;
    }
}

double *compute_next_step(double *new_pos, double dt)
{
    // Compute the next positions of the robots according to their current positions and motor commands
    // This will be used for collision detection (check if they'd be overlapping on the next step)
    // Does NOT address turning that happens when there is a collision
    // new positions are serialized: [x_1, y_1, theta_1, x_2, y_2, theta_2, x_3, ...]

#pragma omp for schedule(static)
    for (auto &r : robots)
    {

        double theta = r->pos[2];
        double x = r->pos[0];
        double y = r->pos[1];
        double temp_x = x;

        double temp_y = y;
        double temp_cos, temp_sin, phi;
        switch (r->motor_command)
        {
        case 1:
        { // forward
            //theta += r->motor_error * dt;
            double speed = r->forward_speed * dt;
            temp_x = speed * cos(theta) + r->pos[0];
            temp_y = speed * sin(theta) + r->pos[1];
            break;
        }
        case 2:
        { // CW rotation
            phi = -r->turn_speed * dt;
            theta += phi;
            temp_cos = radius * cos(theta + 4 * PI / 3);
            temp_sin = radius * sin(theta + 4 * PI / 3);
            temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
            temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
            break;
        }
        case 3:
        { // CCW rotation
            phi = r->turn_speed * dt;
            theta += phi;
            temp_cos = radius * cos(theta + 2 * PI / 3);
            temp_sin = radius * sin(theta + 2 * PI / 3);
            temp_x = x + temp_cos - temp_cos * cos(phi) + temp_sin * sin(phi);
            temp_y = y + temp_sin - temp_cos * sin(phi) - temp_sin * cos(phi);
            break;
        }
        }

        new_pos[i * 3] = temp_x;
        new_pos[i * 3 + 1] = temp_y;
        new_pos[i * 3 + 2] = wrap_angle(theta);
    }
    return new_pos;
}

int find_collisions(double *new_pos, int self_id, int time)
{
    // Check to see if motion causes robots to collide with their updated positions
    double self_x = new_pos[self_id * 3];
    double self_y = new_pos[self_id * 3 + 1];

    // Check for collision with wall
    if (self_x <= radius || self_x >= arena_width - radius || self_y <= radius || self_y >= arena_height - radius)
    {
        return 2;
    }
    bool abort = false;

    //#pragma omp for
    for (int other_id = 0; other_id < num_robots; other_id++)
    {
        //#pragma omp flush (abort)
        if (!abort)
        {
            if (other_id != self_id)
            { // Don't compare to self
                double other_x = new_pos[other_id * 3];
                double other_y = new_pos[other_id * 3 + 1];
                // Get distance to other robots
                double dist_x = self_x - other_x;
                double dist_y = self_y - other_y;
                double distance = sqrt(pow(dist_x, 2) + pow(dist_y, 2));

                // Check if new positions are intersecting
                if (distance < 2 * radius)
                {
                    abort = true;
                    //#pragma omp flush (abort)
                }
            }
        }
    }
    if (abort)
    {
        return 1;
    }
    else
    {
        // Otherwise no collisions
        return 0;
    }
}

bool run_simulation_step()
{
    static int tick = 0;
    tick++;

    total_secs = tick / SECOND;
    double dt = 1.0 / SECOND; // Time change for speed determination

    int secs = total_secs % 60;
    int mins = (total_secs / 60) % 60;
    int hours = total_secs / 3600;
    sprintf(rt, "%02d:%02d:%02d", hours, mins, secs);

    int i, j;

    //double rotation_step = .025; //motion step size
    double rotation_step = .05; //motion step size

    double *new_pos = (double *)malloc(num_robots * 3 * sizeof(double));

#pragma omp parallel
    {

//run a step of most or all robot controllers
#pragma omp for schedule(static)
        for (auto &r : robots)
        {
            //printf("%d\n", i);
            //run controller this time step with p_control_execute probability
            if ((rand()) < (int)(p_control_execute * RAND_MAX))
            {
                r->robot_controller();
            }
        }

        // COMMUNICATION
        // Only communicate at tick rate achievable by kilobots (simulate CSMA/CD)
        if (tick % comm_rate == 0)
        {
            seed = (rand() % shuffles) * num_robots;
#pragma omp for
            for (int t = 0; t < num_robots; t++)
            {
                // Loop over all transmitting robots
                int tx_id = order[seed + t];
                Robot *tx_r = robots[tx_id];
                void *msg = tx_r->get_message();
                if (msg)
                {
                    for (int rx_id = 0; rx_id < num_robots; rx_id++)
                    {
                        // Loop over receivers if transmitting robot is sending a message
                        Robot *rx_r = robots[rx_id];
                        if (tx_id != rx_id)
                        {
                            // Check communication range in both directions (due to potentially noisy communication range)
                            double dist = tx_r->distance(tx_r->pos[0], tx_r->pos[1], rx_r->pos[0], rx_r->pos[1]);
                            if (tx_r->comm_out_criteria(dist) && rx_r->comm_in_criteria(dist, msg))
                            {
                                rx_r->received();
                            }
                        }
                    }
                }
            }
        }

        // MOVEMENT
        // Get a temporary next position for all of the robots (parallelized)
        compute_next_step(new_pos, dt);

// Check for collisions using the new positions
#pragma omp for
        for (int r_id = 0; r_id < num_robots; r_id++)
        {
            Robot *r = robots[r_id];
            double new_x = new_pos[r_id * 3];
            double new_y = new_pos[r_id * 3 + 1];
            double new_theta = new_pos[r_id * 3 + 2];

            int collision_type = find_collisions(new_pos, r_id, tick);

            if (collision_type == 0)
            { // No collision
                r->pos[0] = new_x;
                r->pos[1] = new_y;
                r->collision_timer = 0;
            }
            else if (collision_type == 1)
            { // Hitting another kilobot
                if (r->collision_turn_dir == 0)
                {
                    new_theta = r->pos[2] - r->turn_speed * dt; // left/CCW
                }
                else
                {
                    new_theta = r->pos[2] + r->turn_speed * dt; // right/CW
                }
                if (r->collision_timer > r->max_collision_timer)
                { // Change turn dir
                    r->collision_turn_dir = (r->collision_turn_dir + 1) % 2;
                    r->collision_timer = 0;
                }
                r->collision_timer++;
            }
            r->pos[2] = wrap_angle(new_theta);
            // If a bot is touching the wall (collision_type == 2), update angle but not position
        }
    }

    static int lastsec = -1;
    bool result = false;

    return tick % draw_delay == 0;
}

void drawFilledCircle(GLfloat x, GLfloat y, GLfloat radius)
{
    int i;
    int triangleAmount = 100; //# of triangles used to draw circle

    //GLfloat radius = 0.8f; //radius
    GLfloat twicePi = 2.0f * PI;

    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(x, y); // center of circle
    for (i = 0; i <= triangleAmount; i++)
    {
        glVertex2f(
            x + (radius * cos(i * twicePi / triangleAmount)),
            y + (radius * sin(i * twicePi / triangleAmount)));
    }
    glEnd();
}

void drawFilledTriangle(GLfloat x, GLfloat y, double rad, double theta)
{
    // Draw a filled triangle at the specified location with bearing theta
    // size = equilateral triangle of size inscribed by circle with given radius
    glBegin(GL_TRIANGLES);
    glVertex2f(x + rad * cos(theta), y + rad * sin(theta));
    glVertex2f(x + rad * cos(theta + 2 * PI / 3), y + rad * sin(theta + 2 * PI / 3));
    glVertex2f(x + rad * cos(theta + 4 * PI / 3), y + rad * sin(theta + 4 * PI / 3));
    glEnd();
}

void drawFilledSquare(GLfloat x, GLfloat y, double rad, double theta)
{
    // Draw a rectange at the specified location with bearing theta
    // size = square of size inscribed in circle with given radius
    glBegin(GL_QUADS);
    glVertex2f(x + rad * cos(theta - PI / 4), y + rad * sin(theta - PI / 4));
    glVertex2f(x + rad * cos(theta + PI / 4), y + rad * sin(theta + PI / 4));
    glVertex2f(x + rad * cos(theta + 3 * PI / 4), y + rad * sin(theta + 3 * PI / 4));
    glVertex2f(x + rad * cos(theta + 5 * PI / 4), y + rad * sin(theta + 5 * PI / 4));
    glEnd();
}

void draw_scene(void)
{
    // Drawing routine
    static int snapshottaken = 0;
    static bool takesnapshot = false;
    //draws the arena

    takesnapshot = run_simulation_step();

    if (takesnapshot)
    {
        //Background
        glColor3f(.15, .15, .15);
        glRectd(0, 0, arena_width, arena_height);

        // Draw projected shapes (background)
        for (int i = 0; i < polygons.size(); i++)
        {
            std::vector<float> c = polygons[i].color;
            glColor3f(c[0] * .6, c[1] * .6, c[2] * .6);
            glBegin(GL_POLYGON);
            for (int j = 0; j < polygons[i].points.size(); j++)
            {
                glVertex2f(polygons[i].points[j].x, polygons[i].points[j].y);
            }
            glEnd();
        }
        for (int i = 0; i < rects.size(); i++)
        {
            std::vector<float> c = rects[i].color;
            glColor3f(c[0] * .6, c[1] * .6, c[2] * .6);
            glRectd(rects[i].pos.x, rects[i].pos.y, rects[i].pos.x + rects[i].width, rects[i].pos.y + rects[i].height);
        }
        glColor3f(.6, 0.0, 0.0);
        for (int i = 0; i < circles.size(); i++)
        {
            drawFilledCircle(circles[i].x, circles[i].y, circles[i].rad);
        }

        // Borderlands/edge
        glColor3f(.3, .3, .3);
        glRectd(0, 0, edge_width, arena_height);                          // left
        glRectd(0, 0, arena_width, edge_width);                           // Bottom
        glRectd(arena_width - edge_width, 0, arena_width, arena_height);  // right
        glRectd(0, arena_height, arena_width, arena_height - edge_width); // Top

        glutSetWindowTitle(rt);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(1.0);

        for (auto &r : robots)
        {
            // Draw robots in different shapes depending on feature to detect
            glColor4f((GLfloat)r->color[0], (GLfloat)r->color[1], (GLfloat)r->color[2], 1.0);
            // if (r->detect_which_feature == 0)
            // {
            drawFilledCircle((GLfloat)r->pos[0], (GLfloat)r->pos[1], radius);
            // }
            // else if (r->detect_which_feature == 1)
            // {
            //     drawFilledTriangle((GLfloat)r->pos[0], (GLfloat)r->pos[1], radius * 1.3, r->pos[2]);
            // }
            // else if (r->detect_which_feature == 2)
            // {
            //     drawFilledSquare((GLfloat)r->pos[0], (GLfloat)r->pos[1], radius * 1.3, r->pos[2]);
            // }
            // Draw lines for bearing
            glBegin(GL_LINES);
            glColor4f(0.2, 0.2, 0.2, 1.0);
            glVertex2f((GLfloat)r->pos[0], (GLfloat)r->pos[1]);
            glVertex2f((GLfloat)(r->pos[0] + cos(r->pos[2]) * radius), (GLfloat)(r->pos[1] + sin(r->pos[2]) * radius));
            glEnd();
        }

        glFlush();

        glutSwapBuffers();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    if (last)
    {
        //log_info(NULL);
        exit(0);
    }
    if (total_secs >= timelimit)
    {
        last = true;
    }
}

void setup(void)
{
    // Initialization routine.
    for (int i = 0; i < num_robots; i++)
        for (int j = 0; j < shuffles; j++)
            order[i + num_robots * j] = i;

    for (int i = 0; i < num_robots - 1; i++)
        for (int j = 0; j < shuffles; j++)
        {
            int index = j * num_robots + i;
            int r = index + rand() % (num_robots - i);
            int p = order[index];
            order[index] = order[r];
            order[r] = p;
        }
}

void resize_window(int w, int h)
{
    // OpenGL window reshape routine.
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, 100.0, 0.0, 100.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void key_input(unsigned char key, int x, int y)
{
    // Keyboard input processing routine.
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
        zoom = zoom * 1.1;
        break;
    case '+':
        zoom = zoom * 0.9;
        break;
    case '1':
        if (delay > 0)
            delay--;
        printf("busy delay %d\n\r", delay);
        break;
    case '2':
        delay++;
        printf("busy delay %d\n\r", delay);
        break;
    case '3':
        if (draw_delay > 1)
            draw_delay--;
        printf("draw delay %d\n\r", draw_delay);
        break;
    case '4':
        draw_delay++;
        printf("draw delay %d\n\r", draw_delay);
        break;
    default:
        break;
    }
}

void on_idle(void)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-zoom + view_x, view_x, -zoom + view_y, view_y, 0.0f, 1.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    glutPostRedisplay();
}

int main(int argc, char **argv)
{
    // OpenMP settings
    if (num_threads != 0)
    {
        omp_set_dynamic(0);               // Explicitly disable dynamic teams
        omp_set_num_threads(num_threads); // Use num_threads for all consecutive parallel regions
    }

    // Get shapes from file
    if (!rects_filename_base.empty())
    {
        std::string rects_filename = shapes_dir + "/" + rects_filename_base + "-" +
                                     std::to_string(arena_rows) + "x" + std::to_string(arena_rows) +
                                     "-[" + float_to_string(color_fill_ratio[0]) + "," +
                                     float_to_string(color_fill_ratio[1]) + "," + float_to_string(color_fill_ratio[2]) +
                                     "]-" +
                                     std::to_string(trial_num) + ".txt";
        std::cout << "Using rectangles:\t" << rects_filename << std::endl;
        rects = gen_color_rects(rects_filename);
    }
    if (!circles_filename_base.empty())
    {
        std::string circles_filename = shapes_dir + "/" + circles_filename_base + "-" +
                                       float_to_string(color_fill_ratio[0]) + "-" +
                                       "r" + std::to_string(circles_radius) + "-" +
                                       std::to_string(trial_num) + ".txt";
        std::cout << "Using circles:\t" << circles_filename << std::endl;
        circles = gen_color_circles(circles_filename);
    }
    if (!polys_filename_base.empty())
    {
        std::string polys_filename = shapes_dir + "/" + polys_filename_base + "-" +
                                     float_to_string(color_fill_ratio[0]) + "-" +
                                     std::to_string(trial_num) + ".txt";
        std::cout << "Using polygons:\t" << polys_filename << std::endl;
        polygons = gen_color_polys(polys_filename);
    }

    // DEBUGGING
    // Print stuff here to test parameters/variables
    //robots = (Robot **)malloc(num_robots * sizeof(Robot *)); //creates an array of robots
    order = (int *)malloc(shuffles * num_robots * sizeof(int));
    //seed random variable for different random behavior every time
    unsigned int t = 0;

    if (seed)
    {
        t = seed;
    }
    else
    {
        t = (unsigned int)time(NULL);
    }

    srand(t);

    //set the simulation time to 0
    time_sim = 0;

    //inital zoom and scroll positions
    zoom = arena_width;
    view_x = arena_width;
    view_y = arena_height;

    setup();

    //do some open gl stuff
    for (int i = 0; i < radius; i++)
    {
        ch[i] = sqrt(radius * radius - i * i);
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
    else
    {
        while (total_secs < timelimit)
        {
            run_simulation_step();
        }
    }
    return 0;
}
