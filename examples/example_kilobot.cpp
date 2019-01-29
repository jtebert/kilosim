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
  }

  void loop()
  {
  }
};
} // namespace Kilosim