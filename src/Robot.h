#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include "LightPattern.h"

const double motion_error_std = .02;
const double PI = 3.14159265358979324;
const uint32_t GAUSS = 10000;
const uint8_t right = 2;
const uint8_t left = 3;
const uint8_t sensor_lightsource = 1;
const uint8_t RADIUS = 16;
const uint8_t X = 0;
const uint8_t Y = 1;
const uint8_t T = 2;

#define SECOND 32

struct rgb
{
	double red, green, blue;
};

namespace KiloSim
{

/*!
 * This matches the Kilobot Library API. For detailed documentation and usage,
 * see the [Kilolib documentation](https://www.kilobotics.com/docs/index.html).
 */
class Robot
{
  protected:
	//! World the robot belongs to (used for getting light pattern data)
	LightPattern *m_light_pattern;

  public:
	uint16_t id;
	//x, y, theta position in real world, don't use these in controller, that's cheating!!
	double pos[3];
	//value of how motors differ from ideal, don't use these, that's cheating!!
	double motor_error;
	// communication range between robots (mm)
	double comm_range = 6 * 16;
	// RGB LED display color, values 0-1
	double color[3];

	uint8_t collision_turn_dir;
	uint32_t collision_timer;
	uint32_t max_collision_timer;

	//robot commanded motion 1=forward, 2=cw rotation, 3=ccw rotation, 4=stop
	int motor_command;
	virtual void set_color(rgb c)
	{
		color[0] = c.red;
		color[1] = c.green;
		color[2] = c.blue;
	}

	// Flag set to 1 when robot wants to transmit
	int tx_request;

	// Flag set to 1 when new message received
	int incoming_message_flag;

	virtual void *get_message() = 0;

	double forward_speed = 24; // mm/s
	double turn_speed = 0.5;   // rad/s

	double battery = -1;

  public:
	/*!
	 * Must implement an robot initialization
	 * **IMPORTANT!** Things break (with LightPatterns) if you try to call this
	 * *before* adding a Robot to a World.
	 */
	void robot_init(double, double, double);
	virtual void init() = 0;

	//! Add a pointer to the world that the robot is part of
	void add_light(LightPattern *light_pattern);

	// Robot's internal timer
	int timer;

	// Must implement the controller
	void robot_controller();
	virtual void controller() = 0;

	virtual void sensing(int, int[], int[], int[], int[]) = 0;

	virtual char *get_debug_info(char *buffer, char *rt) = 0;

	virtual double comm_out_criteria(double dist) = 0;
	virtual bool comm_in_criteria(double dist, void *cd) = 0;

	// Useful
	static double distance(double x1, double y1, double x2, double y2)
	{
		double x = x1 - x2;
		double y = y1 - y2;
		double s = pow(x, 2) + pow(y, 2);
		return sqrt(s);
	}

	virtual void received() = 0;
};
} // namespace KiloSim
#endif
