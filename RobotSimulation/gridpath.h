#ifndef GRIDPATH_H
#define GRIDPATH_H

#include <vector>
#include <GL/freeglut.h>

struct GridPath {
	GridPath(size_t GW, size_t GH) : GW(GW), GH(GH), GP(std::vector<char>(GW * GH)) {
		createGridPath();
	}

	size_t GW;
	size_t GH;

	std::vector<char> GP;
	std::vector<cv::Point2f> landmarks;

	void pushLandMark(float x, float y) {
	    landmarks.push_back(cv::Point2f(x, y));
	}

	void getClosestLandmark(const cv::Point2f& x) {
		for (int i = 0; i < landmarks.size(); ++i) {

		}
	}

	void line(float x0, float y0, float x1, float y1) {
		for (int y = 0; y <2; ++y) {
			for (int x = 0; x < 2; ++x) {
				float xp0 = x0 + x;
				float yp0 = y0 + y;
				float xp1 = x1 + x;
				float yp1 = y1 + y;

				float dx = xp1 - xp0;
				float dy = yp1 - yp0;
				const int R = 1000;
				float dsx = dx / R;
				float dsy = dy / R;
				for (int i = 0; i < R; ++i) {
					int xx = xp0 + i * dsx;
					int yy = yp0 + i * dsy;
					if (yy * GW + xx >= 0 && yy * GW + xx < GW * GH)
						GP[yy * GW + xx] = 1;
				}
			}
		}
	}

	void line(const cv::Point2f& a, const cv::Point2f& b) {
		line(a.x, a.y, b.x, b.y);
	}

	void createGridPath() {
		pushLandMark(0, 0);
		pushLandMark(0, 60);
		for (int i = 0; i < landmarks.size() - 1; ++i)
			line(landmarks[i], landmarks[i + 1]);
	}

	void render(int Width, int Height) {
		float x0 = 0;// -(Width / 2.0f);
		float y0 = 0;// -(Height / 2.0f);
		float dx = Width / float(GW);
		float dy = Height / float(GH);
		glPushMatrix();
		glTranslatef(-1, 0, 1);
		glTranslatef(x0, 0, y0);


		// boundaries 
		//glTranslatef(0, 0.1f, 0);
		glColor3f(0.4f, 0, 0);
		glBegin(GL_LINE_LOOP);
		glVertex3f(-Width / 2.0f, 0, -Height / 2.0f);
		glVertex3f(Width / 2.0f, 0, -Height / 2.0f);
		glVertex3f(Width / 2.0f, 0, Height / 2.0f);
		glVertex3f(-Width / 2.0f, 0, Height / 2.0f);
		glEnd();
		
		// path
		for (size_t i = 0; i < GH; ++i) {
			for (size_t j = 0; j < GW; ++j) {
				if (GP[i * GW + j] == 1) {
					glColor3f(0, 0.2, 0.5);
					glBegin(GL_QUADS);
					glVertex3f(j * dx, 0, i * dy); glVertex3f((j + 1) * dx, 0, i * dy);
					glVertex3f((j + 1) * dx, 0, (i + 1) * dy); glVertex3f(j * dx, 0, (i + 1) * dy);
					glEnd();
				}
			}
		}

		// floor
		glTranslatef(0, -0.5, 0);
		glColor3f(0.9, 0.85, 0.4);
		glBegin(GL_QUADS);
		glVertex3f(-Width * 2.0f, 0, -Height * 2.0f);
		glVertex3f(Width * 2.0f, 0, -Height * 2.0f);
		glVertex3f(Width * 2.0f, 0, Height * 2.0f);
		glVertex3f(-Width * 2.0f, 0, Height * 2.0f);
		glEnd();
		glPopMatrix();
	}
};


#endif // GRIDPATH_H