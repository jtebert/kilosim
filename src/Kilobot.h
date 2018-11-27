#pragma once
#ifndef KILOLIB_H
#define KILOLIB_H
#undef RGB

#include <math.h>
#include <omp.h>
//#include "KiloSim.h"
#include "Robot.h"

namespace KiloSim
{

const uint8_t ir = 1;
const uint8_t NORMAL = 1;
const uint8_t tolerance = 60;

typedef double distance_measurement_t;

//communication data struct without distance it should be 9 bytes max
struct message_t
{
	unsigned char type = 0;
	unsigned char data[9];
	unsigned char crc;
};

/*!
 * The abstract class Kilobot provides the implementation of the functions and
 * attributes given by the
 * [Kilolib](https://www.kilobotics.com/docs/index.html). You can imagine this
 * as standing in for the physical Kilobots that your code will run on.
 *
 * Your implementation of Kilobot code should be in a class that inherits from
 * the Kilobot class. It must implement the methods `setup()` and `loop()`.
 * Unlike when using the Kilolib, these are automatically used as passed to the
 * `kilo_start` function. Similarly, the following substitutions are made in
 * place of using a main() function in Kilobot:
 *
 * - `kilo_message_rx` => `void message_rx(message_t *m, distance_measurement_t *d)`
 * - `kilo_message_tx` => `message_t *message_tx()`
 * - `kilo_message_tx_success` => `void message_tx_success()`
 *
 * This means that instead of setting these in a `main()` function, you simply
 * implement the righthand methods in your Kilobot class.
 *
 * **NOTE:** Any values (attributes) that you want to be accessible to your
 * aggregator functions must be declared public.
 */
class Kilobot : public Robot
{
  public:
	bool left_ready = false;
	bool right_ready = false;
	int kilo_turn_right = 50;
	int kilo_straight_left = 50;
	int kilo_straight_right = 50;
	int kilo_turn_left = 50;
	int turn_right = 0;
	int turn_left = 0;
	uint32_t kilo_ticks = 0;

	double distance_measurement;
	bool message_sent = false;

	struct rgb
	{
		double red, green, blue;
	};

	rgb RGB(double r, double g, double b)
	{
		rgb c;
		c.red = r;
		c.green = g;
		c.blue = b;
		return c;
	}

	void rand_seed(char seed) {}

	unsigned char rand_soft()
	{
		return rand() * 255 / RAND_MAX;
	}

	uint8_t rand_hard()
	{
		uint8_t x = rand() % 255;
		return x;
	}

	unsigned char message_crc(message_t *m)
	{
		int crc = 0;
		for (int i = 0; i < 9; i++)
		{
			crc += m->data[i];
		}
		return crc % 256;
	}

	void set_color(rgb c)
	{
		color[0] = c.red;
		color[1] = c.green;
		color[2] = c.blue;
	}

	virtual void setup() = 0;

	void init()
	{
		double two_hours = SECOND * 60 * 60 * 2;
		battery = (1 + gauss_rand(rand()) / 5) * two_hours;
		setup();
	}

	virtual void loop() = 0;
	void message_rx(message_t *message, distance_measurement_t *distance_measurement){};

	void controller()
	{
		//if (pos[0] > 1000 && pos[0] < 1005 && pos[1]>1000 && pos[1] < 1005)
		//{
		//	distance_measurement = 35;
		//	message_t m;
		//	m.data[0] = 1;
		//	m.data[1] = 5;
		//	m.data[2] = 10;
		//	m.data[3] = 1;
		//	message_rx(&m, &distance_measurement);
		//}
		if (message_sent)
		{
			tx_request = 0;
			message_sent = false;
			message_tx_success();
		}
		kilo_ticks++;
		int rand_tick = rand();
		if (rand_tick < RAND_MAX * .1)
		{
			if (rand_tick < RAND_MAX * .05)
				kilo_ticks--;
			else
				kilo_ticks++;
		}
		this->loop();
		motor_command = 4;
		if (right_ready && turn_right == kilo_turn_right)
		{
			motor_command -= 2;
		}
		else
		{
			right_ready = false;
		}
		if (left_ready && turn_left == kilo_turn_left)
		{
			motor_command -= 1;
		}
		else
		{
			left_ready = false;
		}
		if (message_tx())
			tx_request = ir;
		else
			tx_request = 0;
	}

	void kilo_init() {}

	void spinup_motors()
	{
		left_ready = true;
		right_ready = true;
	}

	void set_motors(char l, char r)
	{
		turn_left = l;
		turn_right = r;
	}

	int16_t get_ambientlight()
	{
		// Get point at front/nose of robot
		int pos_x = pos[0] + RADIUS * 1 * cos(pos[2]);
		int pos_y = pos[1] + RADIUS * 1 * sin(pos[2]);
		// Get the 10-bit light intensity from the robot
		//printf("here\n");
		//return m_world->get_light(pos_x, pos_y);
	}

	void delay(int i) {}

	double comm_out_criteria(double dist)
	{
		//standard circular transmission area
		if (dist > comm_range)
			return 0; // it's outside communication range
		return dist;
	}

	bool comm_in_criteria(double dist, void *cd)
	{
		distance_measurement = dist;
		message_rx((message_t *)cd, &distance_measurement);
		return true;
	}

	unsigned char estimate_distance(distance_measurement_t *d)
	{
		if (*d < 255)
			return (unsigned char)*d;
		else
			return 255;
	}

	void sensing(int features, int type[], int x[], int y[], int value[]) {}

	void message_tx_success(){};
	message_t *message_tx(){};

	void *get_message()
	{
		void *m = this->message_tx();
		if (m)
		{
			this->message_tx_success();
		}
		return m;
	}

	void received()
	{
		message_sent = true;
	}

	char *get_debug_info(char *buffer, char *rt)
	{
		return buffer;
	}
};
/*! \example example_kilobot.cpp
 * Example of a minimal custom Kilobot implementation
 */
} // namespace KiloSim

#endif