#include "dart/simulation/simulation.hpp"
#include "MyWindow.h"
using namespace dart::simulation;

int main(int argc,char** argv)
{
	WorldPtr world = std::make_shared<World>();
	MyWindow window(world);
	glutInit(&argc, argv);
	window.initWindow(1280,720, "Sim");
	glutMainLoop();
	return 0;
}