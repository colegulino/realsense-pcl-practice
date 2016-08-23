// 
// Utilities for rendering point cloud data
// 
#include <render_utils/render_utils.h>

namespace render_utils
{
	// 
	// Point cloud window
	// 
	point_cloud_window::point_cloud_window(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud)
		: point_cloud_(point_cloud)
	{
		glfwInit();
		win_ = glfwCreateWindow(1280, 960, "Point Cloud", nullptr, nullptr);
		glfwSetCursorPosCallback(win_, on_cursor_pos);
		glfwSetMouseButtonCallback(win_, on_mouse_button);
		glfwMakeContextCurrent(win_); 
	}

	void point_cloud_window::render()
	{
		glfwPollEvents();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
    	// gluPerspective(60, (float)1280/960, 0.01f, 20.0f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
    	// gluLookAt(0,0,0, 0,0,1, 0,-1,0);
		glTranslatef(0,0,+0.5f);
		glRotated(pitch, 1, 0, 0);
		glRotated(yaw, 0, 1, 0);
		glTranslatef(0,0,-0.5f);
		glPointSize(2);
		glEnable(GL_DEPTH_TEST);

		glBegin(GL_POINTS);
		for(unsigned int i = 0; i < point_cloud_->size(); i++)
		{
			auto point = point_cloud_->at(i);
			glColor3ub(point.r, point.g, point.b);
			glVertex3f(point.x, point.y, point.z);
		}
		glEnd();

		glfwSwapBuffers(win_);
	}

	void on_mouse_button(GLFWwindow* win, int button, int action, int mods)
	{
		if(button == GLFW_MOUSE_BUTTON_LEFT)
		{
			ml = action == GLFW_PRESS;
		}
	}

	double clamp(double val, double lo, double hi)
	{
		return val < lo ? lo : val > hi ? hi : val;
	}

	void on_cursor_pos(GLFWwindow* win, double x, double y)
	{
		if(ml)
		{
			yaw = clamp(yaw - (x - lastX), -120, 120);
			pitch = clamp(pitch + (y - lastY), -80, 80);
		}
		lastX = x;
		lastY = y;
	}

}