#include "KiloSim.h"
#include <stdio.h>
#include <stdlib.h>

// Implementation of Kilobot Arena/World

namespace KiloSim
{
World::World(double arenaWidth, double arenaHeight) : arenaWidth(arenaWidth),
                                                      arenaHeight(arenaHeight) {}

bool World::hasLightPattern()
{
    return !lightPattern.data.empty();
}
} // namespace KiloSim