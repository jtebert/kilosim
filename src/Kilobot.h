#ifndef KILOLIB_H
#define KILOLIB_H
#undef RGB

#include <math.h>
#include <omp.h>
#include "Robot.h"

namespace Kilosim
{

const uint8_t NORMAL = 1;

typedef double distance_measurement_t;

//! [Kilolib API] Communication data struct without distance (should be 9 bytes max).
struct message_t
{
	//! Type of the message (currently only option is NORMAL)
	uint8_t type = 0;
	//! Message payload (9 bytes)
	uint8_t data[9];
	//! Message crc for validity check
	uint16_t crc;
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
 * @note Any values (attributes) that you want to be accessible to your
 * aggregator functions must be declared public.
 */
class Kilobot : public Robot
{
  private:
	//! Is the left motor ready to move? (aka used spinup_motors())
	bool left_ready = false;
	//! Is the right motor ready to move? (aka used spinup_motors())
	bool right_ready = false;
	//! Set duty cycle of the right motor
	int m_turn_right = 0;
	//! Set duty cycle of the left motor
	int m_turn_left = 0;
	//! Communication range between robots in mm (3 bodylengths)
	const double m_comm_range = 6 * 16;

	double distance_measurement;
	bool message_sent = false;

  protected:
	//! [Kilolib API] Kilobot clock variable
	uint32_t kilo_ticks = 0;
	//! [Kilolib API] Calibrated straight (left motor) duty cycle
	const int kilo_straight_left = 50;
	//! [Kilolib API] Calibrated straight (right motor) duty cycle
	const int kilo_straight_right = 50;
	//! [Kilolib API] Calibrated turn left duty cycle
	const int kilo_turn_left = 50;
	//! [Kilolib API] Calibrated turn right duty cycle
	const int kilo_turn_right = 50;

  private:
	/***************************************************************************
	 * REQUIRED ROBOT CONTROL FUNCTIONS
	 **************************************************************************/

	void init()
	{
		// Set the Kilobot's battery level (done here because actual battery
		// life is specific to the Kilobots and not a general property of Robots)
		double two_hours = SECOND * 60 * 60 * 2;
		battery = (1 + gauss_rand(rand()) / 5) * two_hours;
		setup();
	}

	void controller()
	{
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
		m_motor_command = 4;
		if (right_ready && m_turn_right == kilo_turn_right)
		{
			m_motor_command -= 2;
		}
		else
		{
			right_ready = false;
		}
		if (left_ready && m_turn_left == kilo_turn_left)
		{
			m_motor_command -= 1;
		}
		else
		{
			left_ready = false;
		}
		if (message_tx())
			tx_request = 1;
		else
			tx_request = 0;
	}

  protected:
	/***************************************************************************
	 * REQUIRED USER API FUNCTIONS
	 **************************************************************************/

	/*!
	 * [User API] User-implemented setup function that is run once in initialization
	 */
	virtual void setup() = 0;
	/*!
	 * [User API] User-implemented loop function that is called for the Kilobot on every tick
	 */
	virtual void loop() = 0;

	/***************************************************************************
	 * OPTIONAL USER API FUNCTIONS
	 **************************************************************************/

	/*!
	 * [User API] Function that is called when the Kilobot receives a message
	 * On real robots, this is called as an interrupt, so processing here (outside the loop) should be minimized
	 * @param message Contents of the received message
	 * @param distance_measurement Estimated distance (in mm) from the Kilobot sending the message
	 */
	void message_rx(message_t *message, distance_measurement_t *distance_measurement){};

	/*!
	 * [User API] Produce the message to transmit
	 * By default, it returns NULL, which means no message is transmitted
	 * @return Contents of the sent message
	 */
	message_t *message_tx()
	{
		return NULL;
	};

	/*!
	 * [User API] Callback for successful message transmission
	 * (By default, it does nothing)
	 */
	void message_tx_success(){};

	/***************************************************************************
	 * KILOLIB API FUNCTIONS
	 **************************************************************************/

	/*!
	 * [KiloLib API] Create an RGB color
	 *
	 * @param r Red intensity (0-1)
	 * @param g Green intensity (0-1)
	 * @param b Blue intensity (0-1)
	 */
	rgb RGB(double r, double g, double b)
	{
		rgb c;
		c.red = r;
		c.green = g;
		c.blue = b;
		return c;
	}

	/*!
	 * [Kilolib API] Estimate distance in mm based on signal strength measurements.
	 *
	 * TODO: This isn't used but it's part of the Kilolib API. Not even sure of its accuracy...
	 * @param d Signal strength measurement for a message
	 * @return Positive integer distance estimate in mm
	 */
	uint8_t estimate_distance(distance_measurement_t *d)
	{
		if (*d < 255)
			return (unsigned char)*d;
		else
			return 255;
	}

	/*!
	 * [Kilolib API] Pauses the program for a specified amount of time
	 *
	 * This function receives as an argument a positive 16-bit integer `ms` that
	 * represents the number of milliseconds for which to pause the program
	 *
	 * TODO: Part of the KiloLib API but does nothing (issue: using kilo_ticks for timing in simulation)
	 *
	 * @param ms Number of milliseconds to pause the program (there are 1000
	 * milliseconds in a second).
	 */
	void delay(uint16_t ms) {}

	/*!
	 * [KiloLib API] Compute a cyclic redundancy check for a message
	 * Used as error-detecting code for receiving robot to verify the contents
	 * of the message.
	 * @param m Pointer to the message for which to create a code
	 * @return Byte hashing the message data contents
	 */
	uint16_t message_crc(message_t *m)
	{
		int crc = 0;
		for (int i = 0; i < 9; i++)
		{
			crc += m->data[i];
		}
		return crc % 256;
	}

	/*!
	 * [Kilolib API] Hardware random number generator
	 * TODO: Currently this does the same thing as rand_soft
	 */
	uint8_t rand_hard()
	{
		uint8_t x = rand() % 255;
		return x;
	}

	/*!
	 * [Kilolib API] Software random number generator
	 * TODO: Currently does the same thing as rand_hard
	 */
	uint8_t rand_soft()
	{
		return rand() * 255 / RAND_MAX;
	}

	/*!
	 * [Kilolib API] Seed software random number generator.
	 * TODO: Currently this does nothing
	 */
	void rand_seed(char seed) {}

	/*!
	 * [Kilolib API] Get the 10-bit light intensity from the Kilobot's light sensor (from World's LightPattern)
	 * @return 10-bit monochrome light intensity
	 */
	int16_t get_ambientlight()
	{
		if (m_light_pattern)
		{
			// Get point at front/nose of robot
			int pos_x = pos[0] + RADIUS * 1 * cos(pos[2]);
			int pos_y = pos[1] + RADIUS * 1 * sin(pos[2]);
			// Get the 10-bit light intensity from the robot
			return m_light_pattern->get_ambientlight(pos_x, pos_y);
		}
		else
		{
			printf("ERROR: Cannot get_ambientlight() until Kilobot is added to a World\n");
			exit(EXIT_FAILURE);
		}
	}

	// TODO: Not implementing get_voltage() from Kilolib. Could get it from battery value
	// TODO: Not implementing get_temperature()... and probably don't need to

	/*!
	 * [Kilolib API] Set the rate of both the motors. Set both to go straight
	 * @param l Speed of the motor to turn left
	 * @param r Speed of the motor to turn right
	 */
	void set_motors(char l, char r)
	{
		m_turn_left = l;
		m_turn_right = r;
	}

	/*!
	 * [Kilolib API] Spin up both motors to overcome static friction
	 */
	void spinup_motors()
	{
		left_ready = true;
		right_ready = true;
	}

	/*!
	 * [Kilolib API] Set the Kilobot's LED color
	 * @param c RGB color to set the LED to
	 */
	void set_color(rgb c)
	{
		color[0] = c.red;
		color[1] = c.green;
		color[2] = c.blue;
	}

	bool comm_criteria(double dist)
	{
		// Standard circular transmission area
		return dist <= m_comm_range;
	}

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
} // namespace Kilosim

#endif
