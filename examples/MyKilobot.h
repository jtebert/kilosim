#include <kilosim/Kilobot.h>

#include <iostream>

namespace Kilosim
{
/*!
 * This is a barebones example implementation of a Kilobot class
*/
class MyKilobot : public Kilobot
{
  public:
    // I'm using this variable for checking logging/downcasting
    int16_t light_intensity = -1;

  private:
// Variables
#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

    // Initialize
    message_t transmit_msg;
    int new_message = 0;
    uint32_t last_checked = 0;
    int random_number = 0;
    int dice = 0;
    int curr_motion = 0;
    uint32_t next_check_dur;

    void set_motion(int new_motion)
    {
        if (curr_motion != new_motion)
        {
            curr_motion = new_motion;
            if (new_motion == STOP)
            {
                set_motors(0, 0);
            }
            else if (new_motion == FORWARD)
            {
                spinup_motors();
                set_motors(kilo_straight_left, kilo_straight_right);
            }
            else if (new_motion == LEFT)
            {
                spinup_motors();
                set_motors(kilo_turn_left, 0);
            }
            else
            { // RIGHT
                spinup_motors();
                set_motors(0, kilo_turn_right);
            }
        }
    }

    // REQUIRED KILOBOT FUNCTIONS

    void setup()
    {
        transmit_msg.type = NORMAL;
        transmit_msg.data[0] = 0;
        transmit_msg.crc = message_crc(&transmit_msg);
        set_color(RGB(1, 0, 1));
        set_motion(FORWARD);
        next_check_dur = 0;
    }

    void loop()
    {
        // Example dispersion algorithm
        if (kilo_ticks > last_checked + next_check_dur)
        {
            next_check_dur = ((rand_hard() % 4) + 1) * 32;
            last_checked = kilo_ticks;
            random_number = rand_hard();
            dice = (random_number % 4);

            if (dice <= 1)
            {
                // set_color(RGB(0, 1, 0));
                set_motion(FORWARD);
            }
            else if (dice == 2)
            {
                // set_color(RGB(1, 0, 0));
                set_motion(LEFT);
            }
            else if (dice == 3)
            {
                // set_color(RGB(0, 0, 1));
                set_motion(RIGHT);
            }
            else
            { // Should only happen if there's a problem/mistake
                // set_color(RGB(0, 1, 1));
                set_motion(STOP);
            }
        }
        light_intensity = get_ambientlight();
        if (light_intensity > 700)
        {
            set_color(RGB(0, 1, 0));
        }
        else if (light_intensity < 300)
        {
            set_color(RGB(1, 0, 0));
        }
        else
        {
            set_color(RGB(0, 0, 1));
        }
    }

    // Receiving message
    void message_rx(message_t *msg, distance_measurement_t *dist)
    {
        new_message = 1; // Set the flag to 1 to indicate a new message received
    }

    // Sending message
    message_t *message_tx()
    {
        return &transmit_msg;
    }

    void message_tx_success() {}
};
} // namespace Kilosim