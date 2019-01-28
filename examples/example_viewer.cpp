#include "KiloSim.h"
#include "Viewer.h"

int main(int argc, char *argv[])
{
    KiloSim::World world(1200.0, 1200.0);
    // Construct a Viewer with a pointer to the World you want to draw
    KiloSim::Viewer viewer(world);

    // Run a 10 second simulation
    while (world.get_time() < 10)
    {
        world.step();
        // Draw the current state of the world, up to 144 Hz
        viewer.draw();
    }
    return 0;
}