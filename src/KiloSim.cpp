#include "KiloSim.h"
#include <stdio.h>
#include <stdlib.h>

// Implementation of Kilobot Arena/World

namespace KiloSim
{
World::World(double arenaWidth, double arenaHeight) : arenaWidth(arenaWidth), arenaHeight(arenaHeight)
{
    // TODO: Implement constructor without lightImg
}
World::World(double arenaWidth, double arenaHeight, std::string lightImg) : arenaWidth(arenaWidth), arenaHeight(arenaHeight)
{
    // TODO: Implement constructor with lightImg
}

World::~World()
{
    // TODO: Implement World destructor (any destructors, zB)
}

bool World::hasLightPattern()
{
    return !lightPattern.data.empty();
}

void World::setLightPattern(std::string lightImg)
{
    // TODO: Implement setLightPattern
}

void World::addRobot(Robot &robot)
{
    // TODO: Implement addRobot
}

void World::removeRobot(Robot &robot)
{
    // TODO: Implement removeRobot
}

void World::addLogger(Logger *lgr)
{
    logger = lgr;
}

void World::logState()
{
    logger->logState((double)tick / tickRate, robots);
}

double *World::computeNextStep(double *newPos, double dt)
{
    // TODO: Implement computeNextStep (and maybe change from pointers)
}

int World::findCollisions(double *newPos, int selfID, int time)
{
    // TODO: implement findCollisions (and switch from pointers)
}

void World::drawScene()
{
    // TODO: Implement drawScene
}

} // namespace KiloSim