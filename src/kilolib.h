#pragma once
#ifndef KILOLIB_H
#define KILOLIB_H
#undef RGB

#include <math.h>
#include <omp.h>
#include "robot.h"
#include "vars.h"

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

class kilobot : public Robot
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
	virtual void message_rx(message_t *message, distance_measurement_t *distance_measurement) = 0;

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

	std::vector<uint16_t> get_ambientlight()
	{
		// Use shapes to determine light seen by kilobot
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
		//if (!point_in_polygon(p, arena_bounds)) {
		if (!point_in_rect(p, arena_bounds))
		{
			// out of arena = GRAY
			return {500, 500, 500};
		}
		else
		{
			bool abort = false;
			std::vector<uint16_t> colors;
			// Check if in any polygon
			for (int i = 0; i < polygons.size(); i++)
			{
				if (!abort)
				{
					polygon_c_t poly = polygons[i];
					if (point_in_polygon(p, poly))
					{
						colors = {uint16_t(poly.color[0] * 1024),
								  uint16_t(poly.color[1] * 1024),
								  uint16_t(poly.color[2] * 1024)};
						abort = true;
					}
				}
			}
			// Check if in any circle
			for (int i = 0; i < circles.size(); i++)
			{
				if (!abort)
				{
					circle_c_t circ = circles[i];
					if (point_in_circle(p, circ))
					{
						colors = {uint16_t(circ.color[0] * 1024),
								  uint16_t(circ.color[1] * 1024),
								  uint16_t(circ.color[2] * 1024)};
						abort = true;
					}
				}
			}
			// Check if in any rectangle
			for (int i = 0; i < rects.size(); i++)
			{
				if (!abort)
				{
					rect_c_t rect = rects[i];
					if (point_in_rect(p, rect))
					{
						colors = {uint16_t(rect.color[0] * 1024),
								  uint16_t(rect.color[1] * 1024),
								  uint16_t(rect.color[2] * 1024)};
						abort = true;
					}
				}
			}
			if (abort)
			{
				return colors;
			}
			return {0, 0, 0};
		}
	}

	void sensing(int features, int type[], int x[], int y[], int value[]) {}

	virtual void message_tx_success() = 0;
	virtual message_t *message_tx() = 0;

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

#endif
