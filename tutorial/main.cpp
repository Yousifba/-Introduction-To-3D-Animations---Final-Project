#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

int main(int argc, char* argv[])
{
	Display* disp = new Display(1000, 800, "Wellcome");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer;
	Init(*disp);

	renderer.MultipleViews();
	renderer.init(&viewer);
	disp->SetRenderer(&renderer);

	int arm_length = 10;
	viewer.arm_length = arm_length;
	viewer.sys_init();

	
	


	
	disp->launch_rendering(true);

	delete disp;
}
