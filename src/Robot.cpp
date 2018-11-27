#include <math.h>
#include <iostream>
#include <numeric>
#include "Robot.h"

namespace KiloSim
{
void Robot::robot_controller()
{
	if (battery > 0)
	{
		timer++;
		controller();
		switch (motor_command)
		{
		case 1:
		{
			battery -= .5;
		}
		case 2 | 3:
		{
			battery -= .5;
		}
		}
	}
	else
	{
		forward_speed = 0;
		turn_speed = 0;
		motor_command = 4;
		color[0] = .3;
		color[1] = .3;
		color[1] = .3;
		tx_request = 0;
	}
}

void Robot::robot_init(double x, double y, double t)
{
	// printf("INIT: %f, %f, %f\n", x, y, t);
	// Pick a direction to randomly turn in event of collisions
	collision_turn_dir = rand() % 2;
	collision_timer = 0;
	max_collision_timer = (uint32_t)rand() % (20 * SECOND) + (10 * SECOND) + 1; // Max duration 10-30 seconds
	// Initialize robot variables
	pos[0] = x;
	pos[1] = y;
	pos[2] = t;
	motor_command = 0;
	incoming_message_flag = 0;
	tx_request = 0;
	id = rand();
	rand();
	// Generate CLAMPED motor error (avoid extremes by regenerating)
	motor_error = 100;
	double motor_error_clamp = motion_error_std * 1.1;
	while (abs(motor_error) > motor_error_clamp)
	{
		timer = rand() / 100;
		motor_error = Robot::gauss_rand(timer) * motion_error_std;
	}
	// Add random variation to forward/turn speeds
	double turn_speed_error = 100;
	double turn_speed_error_std = turn_speed * 0.1; // 5% of turn speed
	double turn_speed_error_clamp = turn_speed_error_std * 1.1;
	while (abs(turn_speed_error) > turn_speed_error_clamp)
	{
		timer = rand() / 100;
		turn_speed_error = Robot::gauss_rand(timer) * turn_speed_error_std;
	}
	turn_speed = turn_speed + turn_speed_error;
	double forward_speed_error = 100;
	double forward_speed_error_std = forward_speed * 0.1; // 5% of turn speed
	double forward_speed_error_clamp = forward_speed_error_std * 1.1;
	while (abs(forward_speed_error) > forward_speed_error_clamp)
	{
		timer = rand() / 100;
		forward_speed_error = Robot::gauss_rand(timer) * forward_speed_error_std;
	}
	forward_speed = forward_speed + forward_speed_error;
	init();
}

void Robot::add_light_pattern(sf::Image *light_pattern)
{
	m_light_pattern = light_pattern;
}
}