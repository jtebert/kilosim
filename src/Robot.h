#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include "LightPattern.h"

constexpr double motion_error_std = .02;
constexpr double PI = 3.14159265358979324;
constexpr uint32_t GAUSS = 10000;
constexpr uint8_t right = 2;
constexpr uint8_t left = 3;
constexpr uint8_t sensor_lightsource = 1;
constexpr uint8_t RADIUS = 16;
constexpr uint8_t X = 0;
constexpr uint8_t Y = 1;
constexpr uint8_t T = 2;

#define SECOND 32

namespace Kilosim
{
//! Simple representation of red/green/blue color
struct rgb
{
	//! Red component of RGB color
	double red;
	//! Green component of RGB color
	double green;
	//! Blue component of RGB color
	double blue;
};

struct RobotPose
{
	// x, y, and theta (rotation) of a robot
	//! Robot's x-position
	double x;
	//! Robot's y-position
	double y;
	//! Robot's rotation, where 0 points along x-axis and positive is CCW
	double theta;
	RobotPose() : x(0.0), y(0.0), theta(0.0) {}
	RobotPose(double x, double y, double theta)
		: x(x),
		  y(y),
		  theta(theta) {}
};

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
 *
 * In principle, you could create a non-Kilobot robot with this base class, but
 * this hasn't been tested.
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
	//! How long the robot has been turning this way while colliding
	//! (will time out and switch direction)
	uint32_t m_collision_timer = 0;
	//! How long to turn one way when colliding, before switching
	//! (set randomly in robot_init())
	uint32_t m_max_collision_timer;
	//! Value of how motors differ from ideal.
	//! (Don't use these; that's cheating!) Set in robot_init()
	double m_motor_error;
	//! Robot commanded motion 1=forward, 2=cw rotation, 3=ccw rotation, 4=stop
	int m_motor_command;
	//! Base forward speed in mm/s
	//! (Will be randomized around this in robot_init())
	double m_forward_speed = 24;
	//! Base turning speed in rad/s
	//! (Will be randomized around this in robot_init())
	double m_turn_speed = 0.5;
	// TODO: Shouldn't battery also be set in robot_init()?
	/*!
	 * Battery remaining (to be set in `Kilobot.init()`).
	 * This is decremented by 0.5 every tick in which a motor is running. (No
	 * battery reduction occurs when robots are not moving.) At 32 ticks/sec, a
	 * battery life of 2 hours of constant movement is 230400.
	 *
	 * The default value of -1 signifies an artificially infinite battery life.
	 */
	double battery = -1;
	//! Flag set to 1 when robot wants to transmit
	int tx_request;

public:
	//! UUID of the robot, set in robot_init()
	uint16_t id;
	//! Robot's x-position
	//! (Don't use these in controller; that's cheating! It's public for logging
	//! purposes.)
	double x;
	//! Robot's y-position
	//! (Don't use these in controller; that's cheating! It's public for logging
	//! purposes.)
	double y;
	//! Robot's rotation, where 0 points along x-axis and positive is CCW
	//! (Don't use these in controller; that's cheating! It's public for logging
	//! purposes.)
	double theta;
	//! RGB LED display color, values 0-1 (also used as display color by `Viewer`)
	double color[3];
	//! Communication range between robots in mm (3 bodylengths)
	const double comm_range = 6 * 16;

	//! Flag set to 1 when new message received
	// TODO: This doesn't appear to actually be used anymore. Kill it?
	int incoming_message_flag;

	/*!
	 * Get a void pointer to the message the robot is sending and handle any
	 * callbacks for successful message transmission
	 * @return Pointer to message to transmit
	 */
	virtual void *get_message() = 0;

public:
	virtual ~Robot() = default;

	/*!
	 * Initialize a Robot at a position in the world.
	 *
	 * This also calls the child-specific `init()` function.
	 *
	 * @note Things break (with `LightPattern`s) if you try to call this
	 * *before* adding a `Robot` to a `World`.
	 *
	 * @warning This currently does **not** check if the specified Robot
	 * position is within the arena bounds. Robots placed out-of-bounds will not
	 * produce any errors, but they will be considered constantly in a wall
	 * collision.
	 *
	 * @param x x-position to place the Robot in the World
	 * @param y y-position to place the Robot in the World
	 * @param theta rotation/direction of the Robot in radians
	 * (counterclockwise, where 0 is along positive x-axis)
	 */
	void robot_init(double x, double y, double theta);

	/*!
	 * Run the simulated control of the physical Robot (such as battery, and
	 * color). This also calls the child-specific `controller()`.
	 */
	void robot_controller();

	/*!
	 * Add a pointer to the world that the robot is part of and set the
	 * simulation time step size.
	 *
	 * This is automatically called by the `World` when a Robot is added to the
	 * World.
	 *
	 * @param light_pattern Reference to the World's LightPattern
	 * @param dt Seconds per tick (World's simulation step size)
	 */
	void add_to_world(LightPattern &light_pattern, const double dt);

	// TODO: Not sure what use this timer is useful for?
	//! Robot's internal timer
	int timer;

	/*!
	 * Compute the next position of the Robot as if it doesn't run into
	 * anything, based on its current motor and battery states.
	 *
	 * @note This performs no updates to the Robot, but instead returns this
	 * possible new pose
	 *
	 * @return Vector of (x, y, and wrapped theta) to possibly move t
	 */
	RobotPose robot_compute_next_step() const;

	/*!
	 * Move the Robot according to the collision-ignorant `new_pose` and any
	 * `collision`s.
	 *
	 * @note This uses fast pseudo-physics to handle collisions with walls and
	 * other Robots.
	 *
	 * @param new_pose Collision-ignorant next-step (x, y, theta) computed by
	 * `compute_next_step()`
	 * @param collision Whether there's a collision with a wall (-1), another
	 * Robot (1), or no collision (0)
	 */
	void robot_move(const RobotPose &new_pose, const int16_t &collision);

	virtual char *get_debug_info(char *buffer, char *rt) = 0;

	/*!
	 * This is called by a transmitting robot (tx) to set a flag for calling the
	 * message success callback (message_tx_success)
	 */
	virtual void received() = 0;

	/*!
	 * This is called when a robot (rx) receives a message. It calls some
	 * message handling function (e.g., message_rx) specific to the
	 * implementation.
	 */
	virtual void receive_msg(void *msg, double dist) = 0;

protected:
	/*!
	 * Perform any one-time initialization for the specific implementation of
	 * the Robot, such as setting initial battery levels and calling any
	 * user-implementation setup functions. It is called by `robot_init()`.
	 *
	 * If you want to change the robot's battery life, do so here by setting
	 * the `battery` member variable.
	 */
	virtual void init() = 0;

	/*!
	 * Internal control loop for the specific Robot subclass implementation.
	 * This performs any robot-specific controls such as setting motors,
	 * communication flags, and calling user implementation loop functions.
	 * It is called every simulation time step by `robot_controller()`
	 */
	virtual void controller() = 0;

private:
	//! Wrap an angle to be within [0, 2*pi)
	double wrap_angle(double angle) const;
};
} // namespace Kilosim
#endif
