#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <math.h>
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

namespace Kilosim
{

/*!
 * This class provides an abstract controller interface for robots. It provides
 * functions for movement, communication, and interaction with the simulator
 * World. It is the abstract base class for the Kilosim, and as such is not
 * to be directly constructed. It serves as the parent for the Kilobot class,
 * which provides the [Kilolib](https://www.kilobotics.com/docs/index.html)
 * Kilobot library. In turn, Kilobot serves as the parent class for user
 * implementations of Kilobot code (matching what would be written for actual
 * Kilobot robots.)
 *
 * To summarize:
 *
 * - `Robot`: Controller interface for interacting
 * - `Kilobot`: Implementation of Kilolib, inheriting from `Robot` and serving
 *   as parent class for user code
 */
class Robot
{
  protected:
	//! World the robot belongs to (used for getting light pattern data)
	LightPattern *m_light_pattern;
	//! Time per tick (set when Robot added to World)
	double m_tick_delta_t;
	//! When robots collide, which direction this will turn (0 or 1)
	uint8_t m_collision_turn_dir;
	//! How long the robot has been turning this way while colliding (will time out and switch direction)
	uint32_t m_collision_timer = 0;
	//! How long to turn one way when colliding, before switching (set randomly in robot_init())
	uint32_t m_max_collision_timer;
	//! Value of how motors differ from ideal. (Don't use these; that's cheating!) Set in robot_init()
	double m_motor_error;
	//! Robot commanded motion 1=forward, 2=cw rotation, 3=ccw rotation, 4=stop
	int m_motor_command;
	//! Base forward speed in mm/s (Will be randomized around this in robot_init())
	double m_forward_speed = 24;
	//! Base turning speed in rad/s (Will be randomized around this in robot_init())
	double m_turn_speed = 0.5;
	//! Battery remaining (to be set in Kilobot.init())
	//! TODO: Shouldn't this also be set in robot_init()?
	double battery = -1;
	//! Communication range between robots in mm (3 bodylengths)
	const double m_comm_range = 6 * 16;
	//! Flag set to 1 when robot wants to transmit
	int tx_request;

  public:
	//! UUID of the robot, set in robot_init()
	uint16_t id;
	//! (x, y, theta) position in real world. (Don't use these in controller; that's cheating! It's public for logging purposes.)
	double pos[3];
	// RGB LED display color, values 0-1
	double color[3];

	//! Flag set to 1 when new message received
	// TODO: This doesn't appear to actually be used anymore. Kill it?
	int incoming_message_flag;

	virtual void *get_message() = 0;

  public:
	/*!
	 * Must implement an robot initialization
	 * **IMPORTANT!** Things break (with LightPatterns) if you try to call this
	 * *before* adding a Robot to a World.
	 */
	void robot_init(double, double, double);
	virtual void init() = 0;

	virtual void set_color(rgb c)
	{
		color[0] = c.red;
		color[1] = c.green;
		color[2] = c.blue;
	}

	/*!
	 * Add a pointer to the world that the robot is part of
	 * @light_pattern Reference to the World's LightPattern
	 * @dt Seconds per tick (World's simulation step size)
	 */
	void add_to_world(LightPattern &light_pattern, const double dt);

	//! Robot's internal timer
	// TODO: Not sure what use this is
	int timer;

	// Must implement the controller
	void robot_controller();
	virtual void controller() = 0;
	/*!
	 * Compute the next position of the robot if it doesn't run into anything
	 * @return Vector of (x, y, and wrapped theta) to possibly move t
	 */
	std::vector<double> robot_compute_next_step() const;

	void robot_move(const std::vector<double> &new_pose, const int16_t &collision);

	// TODO: Not used and no idea what it's for
	// virtual void sensing(int, int[], int[], int[], int[]) = 0;

	virtual char *get_debug_info(char *buffer, char *rt) = 0;

	// TODO: Why is this implemented in Kilobot.h instead of Robot.cpp? Also this implementation seems weird/pointless. It's only used by Kilosim.cpp for determining if communication works in both directions (and it's symmetric) WTF. I don't think I wrote this...
	/*!
	 * Determine if another robot is within communication range
	 * @dist Distance between the robots (in mm)
	 * @return 0 if out of range; otherwise distance
	 */
	virtual double comm_out_criteria(double dist) = 0;
	/*!
	 * Check if in communication range?
	 */
	virtual bool comm_in_criteria(double dist, void *cd) = 0;

	// Useful
	static double distance(double x1, double y1, double x2, double y2)
	{
		double x = x1 - x2;
		double y = y1 - y2;
		double s = pow(x, 2) + pow(y, 2);
		return sqrt(s);
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

	virtual void received() = 0;

  protected:
	//! Wrap an angle to be within [0, 2*pi)
	double wrap_angle(double angle) const;

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
} // namespace Kilosim
#endif
