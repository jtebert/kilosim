/*
    This is a barebones example implementation of a Kilobot class
*/

#include "Kilobot.h"
#include <iostream>

namespace Kilosim
{
class MyKilobot : public Kilobot
{
public:
  // Declare any attributes that you want to be accessible to aggregator
  // functions as public.
  int16_t light_intensity = -1;

private:
  // Declare any other (internal) persistent attributes here.

  // REQUIRED KILOBOT FUNCTIONS

  void setup()
  {
    // This code will be run once when you call `robot_init(x, y, theta)`
    // Note: This should only be done AFTER the Robot has been added to a World
  }

  void loop()
  {
    // This code is run once per kilo_tick.
  }

  // OPTIONAL KILOBOT FUNCTIONS

  void message_rx(message_t *message, distance_measurement_t *distance_measurement){
      // This is called when a message is received
      // On real robots, this is called as an interrupt, so processing here
      // (outside the loop) should be minimized
      // Implementing this overrides the default function, which does nothing
  };

  message_t *message_tx()
  {
    // This produces the message to send.
    // Implementing this overrides the default function, which returns NULL
    // (No message transmitted)
  }

  void message_tx_success()
  {
    // This is called when a message is successfully transmitted
    // Implementing this overrides the default function, which does nothing
  }
};
} // namespace Kilosim