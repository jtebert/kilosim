/*
    KiloSim

    Visualizer for the Kilobot simulator to display robots and light pattern
    (OpenGL-based)

    Created 2018-11 by Julia Ebert
*/

#include <GL/glew.h>
#include <GL/freeglut.h>
#include "viewer.h"

namespace KiloSim
{
Viewer::Viewer(World *world) : m_world(world)
{
    int argc = 1;
    char *argv[1] = {(char *)""};
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(0, 0);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0f, 1000, 1000, 0.0f, 0.0f, 1.0f);
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glutCreateWindow("Kilobot Simulator");

    // TODO: Figure out how to do this not with a main loop and using class methods as functions
    // glutDisplayFunc(draw);
    //glutReshapeFunc(resize_window);
    //glutIdleFunc(on_idle);
    // glutKeyboardFunc(key_input);
    // glutMainLoop();
}

void Viewer::draw()
{
    // Background
    glColor3f(.15, .15, .15);
    std::vector<double> dimensions = m_world->getDimensions();
    glRectd(0, 0, dimensions[0], dimensions[1]);

    // TODO: Draw world's lightPattern

    // TODO: Implement this
    for (auto &r : m_world->getRobots())
    {
        drawRobot(r);
        // r->draw();
    }

    glFlush();
    glutSwapBuffers();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Viewer::drawRobot(Robot *robot)
{
    // TODO : Implement this
    // Maybe move this to robot.h (each robot responsible for determining its
    // own representation)
}

void Viewer::drawLightPattern()
{
    // TODO: Implement drawLightPattern()
}
} // namespace KiloSim