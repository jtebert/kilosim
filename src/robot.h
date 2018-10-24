#pragma once
#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <math.h>
#include "vars.h"

const double motion_error_std = .02;
const double PI = 3.14159265358979324;
const uint32_t GAUSS = 10000;
const uint8_t right = 2;
const uint8_t left = 3;
const uint8_t sensor_lightsource = 1;
const uint8_t radius = 16;
const uint8_t X = 0;
const uint8_t Y = 1;
const uint8_t T = 2;

struct rgb
{
	double red, green, blue;
};

class Robot
{
  public:
#define NUM_FEATURES 3
	uint8_t feature_estimate = 127;
	uint8_t pattern_belief[NUM_FEATURES] = {127, 127, 127};
	uint8_t decision[3] = {127, 127, 127};
	uint16_t id;
	double pos[3];				   //x,y,theta position in real world, don't use these in controller, that's cheating!!
	double motor_error;			   //value of how motors differ from ideal, don't use these, that's cheating!!
	double comm_range = comm_dist; //communication range between robots
	double color[3];			   //robot color output, values 0-1
	uint8_t detect_which_feature;  // Index of which pattern feature to detect

	// TEMPORARY: Make variables accessible in main loop for debugging purposes
	uint8_t curr_level;
	uint8_t collision_turn_dir;
	uint32_t collision_timer;
	uint32_t max_collision_timer;
	bool is_retransmit;

	// Pull the arena width/height from main to the class. I don't know if this
	// is actually the proper way to pull this information.
	//printf("%d\n", arena_width);
	//int arena_height = arena_height;
	//int arena_width = arena_width;
	//printf("%d\n", arena_width);

	//robot commanded motion 1=forward, 2=cw rotation, 3=ccw rotation, 4=stop
	int motor_command;
	virtual void set_color(rgb c)
	{
		color[0] = c.red;
		color[1] = c.green;
		color[2] = c.blue;
	}

	double dest[3] = {-1, -1, -1};

	//must implement an robot initialization
	void robot_init(double, double, double);
	virtual void init() = 0;

	//robots internal timer
	int timer;

	//must implement the controller
	void robot_controller();
	virtual void controller() = 0;

	virtual void sensing(int, int[], int[], int[], int[]) = 0;

	//flag set to 1 when robot wants to transmit
	int tx_request;

	//flag set to 1 when new message received
	int incoming_message_flag;

	virtual void *get_message() = 0;

	double forward_speed = 24; // mm/s
	double turn_speed = 0.5;   // rad/s

	double battery = -1;

	virtual char *get_debug_info(char *buffer, char *rt) = 0;

	virtual double comm_out_criteria(double dist) = 0;
	virtual bool comm_in_criteria(double dist, void *cd) = 0;

	//useful
	static double distance(double x1, double y1, double x2, double y2)
	{
		double x = x1 - x2;
		double y = y1 - y2;
		double s = pow(x, 2) + pow(y, 2);
		return sqrt(s);
	}

	static double find_theta(double x1, double y1, double x2, double y2)
	{
		if (x1 == x2)
			return 0;
		double x = x2 - x1;
		double y = y2 - y1;

		if (x >= 0 && y >= 0)
		{
			return atan(y / x);
		}
		if (x < 0 && y < 0)
		{
			return atan(y / x) + PI;
		}
		if (x < 0 && y > 0)
		{
			return atan(abs(x) / y) + PI / 2;
		}
		return atan(x / abs(y)) + PI / 2 * 3;
	}

	static double gauss_rand(int timer)
	{
		static double pseudogaus_rand[GAUSS + 1];
		if (pseudogaus_rand[GAUSS] == 1)
		{
			return pseudogaus_rand[timer % GAUSS];
		}
		for (int i = 0; i < GAUSS; i++)
		{
			pseudogaus_rand[i] = gaussrand();
		};
		pseudogaus_rand[GAUSS] = 1;
		return pseudogaus_rand[timer % GAUSS];
	}

	static double tetha_diff(double t1, double t2)
	{
		double diff = t1 - t2;
		if (diff < -PI)
		{
			diff += 2 * PI;
		}
		else if (diff > PI)
		{
			diff -= 2 * PI;
		}
		return diff;
	}
	virtual void received() = 0;

  private:
	static double gaussrand()
	{
		static double V1, V2, S;
		static int phase = 0;
		double x;

		if (phase == 0)
		{
			do
			{
				double U1 = (double)rand() / RAND_MAX;
				double U2 = (double)rand() / RAND_MAX;

				V1 = 2 * U1 - 1;
				V2 = 2 * U2 - 1;
				S = V1 * V1 + V2 * V2;
			} while (S >= 1 || S == 0);

			x = V1 * sqrt(-2 * log(S) / S);
		}
		else
			x = V2 * sqrt(-2 * log(S) / S);

		phase = 1 - phase;
		return x;
	}
};

#endif
