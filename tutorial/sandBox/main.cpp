#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

int main(int argc, char* argv[])
{
	Display* disp = new Display(1000, 800, "Wellcome");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer;
	Init(*disp);
	renderer.init(&viewer);


	disp->SetRenderer(&renderer);

	int arm_length = 10;
	viewer.arm_length = arm_length;
	viewer.sys_init(1);
	viewer.save_snake();

	renderer.MultipleViews();
	renderer.GetScene()->getTrans().matrix() << 0.025f, 0, 0, 0,
		0, 0.025f, 0, 0,
		0, 0, 0.025f, 0,
		0, 0, 0, 1;
	disp->launch_rendering(true);

	delete disp;
}
