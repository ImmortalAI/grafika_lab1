#define _USE_MATH_DEFINES

#include "Render.h"
#include <cmath>
#include <vector>

#include <Windows.h>
#include <GL\GL.h>
#include <GL\GLU.h>

struct Point
{
	double x, y, z;

	Point() : x(0.), y(0.), z(0.) {}

	Point(double x, double y, double z) : x(x), y(y), z(z) {}

	inline void push()
	{
		glColor3d(x, y, z);
		glVertex3d(x, y, z);
	}
};

void raisePoints(std::vector<Point>& points, double deltaZ) {
	for (size_t i = 0; i < points.size(); i++)
		points[i].z += deltaZ;
}

inline double toDeg(double rad) {
	return (rad / M_PI) * 180;
}

inline double toRad(double deg) {
	return (deg / 180) * M_PI;
}

Point getCenter(Point& p1, Point& p2, Point& p3) {
	double a = p1.x * p1.x + p1.y * p1.y - p2.x * p2.x - p2.y * p2.y;
	double b = 2 * p1.x - 2 * p2.x, c = 2 * p1.y - 2 * p2.y;
	double d = p1.x * p1.x + p1.y * p1.y - p3.x * p3.x - p3.y * p3.y;
	double e = 2 * p1.x - 2 * p3.x, f = 2 * p1.y - 2 * p3.y;

	double y0 = (d - (a * e) / b) / (f - (c * e) / b);
	double x0 = (a - y0 * c) / b;

	return Point(x0, y0, p1.z);
}

std::vector<Point> pointsOnCircle(Point& centerP, Point& startP, Point& endP) {
	Point startP1(startP.x - centerP.x, startP.y - centerP.y, startP.z);
	Point endP1(endP.x - centerP.x, endP.y - centerP.y, endP.z);
	double circleR = sqrt(startP1.x * startP1.x + startP1.y * startP1.y);
	double startAngle = acos(startP1.x / circleR), endAngle = acos(endP1.x / circleR);
	if (asin(startP1.y / circleR) < 0)
		startAngle *= -1;
	if (asin(endP1.y / circleR) < 0)
		endAngle *= -1;
	double startAngleDeg = toDeg(startAngle), endAngleDeg = toDeg(endAngle);
	int times = abs((int)(endAngleDeg - startAngleDeg - 1) / 8);
	double deltaRad = (endAngle - startAngle) / (times);

	std::vector<Point> points;
	for (int i = 1; i < times; i++)
	{
		points.push_back(Point(centerP.x + circleR * cos(startAngle + deltaRad * i), centerP.y + circleR * sin(startAngle + deltaRad * i), centerP.z));
	}

	return points;
}

void rotatePoints(std::vector<Point>& points, Point& centerP, double deltaDeg) {
	double deltaRad = toRad(deltaDeg);

	for (size_t i = 0; i < points.size(); i++)
	{
		Point localPoint(points[i].x - centerP.x, points[i].y - centerP.y, centerP.z);
		double distance = sqrt((localPoint.x) * (localPoint.x) + (points[i].y - centerP.y) * (points[i].y - centerP.y));
		double startAngle = acos(localPoint.x / distance) * ((asin(localPoint.y / distance) < 0) ? -1 : 1);
		points[i].x = centerP.x + distance * cos(startAngle + deltaRad);
		points[i].y = centerP.y + distance * sin(startAngle + deltaRad);
	}
}

std::vector<Point> newLayer(std::vector<Point>& model, Point& centerPoint, double rotateDeg, double deltaZ) {
	std::vector<Point> top;

	top = model;
	raisePoints(top, deltaZ);
	rotatePoints(top, centerPoint, rotateDeg);

	glBegin(GL_TRIANGLE_STRIP);

	for (size_t kk = 0; kk < top.size(); kk++)
	{
		model[kk].push();
		top[kk].push();
	}
	model[0].push();
	top[0].push();

	glEnd();
	return top;
}

bool firstRun = true;

void drawmodel(std::vector<Point>& model, std::vector<Point>& circle1, std::vector<Point>& circle2)
{
	glBegin(GL_TRIANGLES);

	model[0].push();
	model[6].push();
	model[7].push();

	model[0].push();
	model[3].push();
	model[6].push();

	model[3].push();
	model[5].push();
	model[6].push();

	model[0].push();
	model[2].push();
	model[3].push();

	glEnd();

	glBegin(GL_TRIANGLE_FAN);

	model[3].push();
	model[5].push();
	for each (auto point in circle1)
		point.push();
	model[4].push();

	glEnd();

	glBegin(GL_TRIANGLE_FAN);

	model[0].push();
	model[1].push();
	for each (auto point in circle2)
		point.push();
	model[2].push();

	glEnd();
}

void Render(double delta_time)
{
	static Point centerCircle;
	static std::vector<Point> model;
	static Point centeredPoint;

	if (firstRun) {
		centerCircle = getCenter(Point(3, 2, 0), Point(7, 2, 0), Point(5, 4, 0));
		model.push_back(Point(0, 0, 0));
		model.push_back(Point(-3, 7, 0));
		model.push_back(Point(3, 9, 0));
		model.push_back(Point(2, 0, 0));
		model.push_back(Point(9, -3, 0));
		model.push_back(Point(5, -6, 0));
		model.push_back(Point(1, -2, 0));
		model.push_back(Point(-6, -5, 0));
		double tempX = 0., tempY = 0.;
		for (size_t i = 0; i < model.size(); i++)
		{
			tempX += model[i].x;
			tempY += model[i].y;
		}
		centeredPoint = Point(tempX / model.size(), tempY / model.size(), 0);
		firstRun = false;
	}

	static Point centreCircle1((model[5].x + model[4].x) / 2, (model[5].y + model[4].y) / 2, 0);
	static std::vector<Point> circle1 = pointsOnCircle(centreCircle1, model[5], model[4]);
	static Point randomPoint(0.5, 7.5, 0);
	static Point centerCircle2 = getCenter(model[1], model[2], randomPoint);
	static std::vector<Point> circle2 = pointsOnCircle(centerCircle2, model[1], model[2]);

	glBegin(GL_QUADS);
	glColor3d(173. / 255, 165. / 255, 135. / 255);
	glVertex3d(-100., -100., 0);
	glVertex3d(-100., 100., 0);
	glVertex3d(100., 100., 0);
	glVertex3d(100., -100., 0);
	glEnd();

	drawmodel(model, circle1, circle2);

	std::vector<Point> underLayer = model;

	underLayer.insert(underLayer.begin() + 2, circle2.begin(), circle2.end());
	underLayer.insert(underLayer.end() - 3, circle1.rbegin(), circle1.rend());
	static size_t layers = 64;
	static double degreeRotate = 0;
	static double deltaDegreeRotate = 0.002;
	static double maxDegreeRotate = 90;
	static double elevateZ = 0.15;

	static bool isScrolling = true;
	if (isScrolling)
	{
		degreeRotate += deltaDegreeRotate;
		if (degreeRotate * layers >= maxDegreeRotate)
			isScrolling = false;
	}
	else
	{
		degreeRotate -= deltaDegreeRotate;
		if (degreeRotate <= 0)
			isScrolling = true;
	}

	for (size_t i = 0; i < layers; i++)
	{
		underLayer = newLayer(underLayer, centeredPoint, degreeRotate, elevateZ);
	}

	std::vector<Point> modelTop = model, circle1Top = circle1, circle2Top = circle2;
	raisePoints(modelTop, elevateZ * layers);
	raisePoints(circle1Top, elevateZ * layers);
	raisePoints(circle2Top, elevateZ * layers);
	rotatePoints(modelTop, centeredPoint, degreeRotate * layers);
	rotatePoints(circle1Top, centeredPoint, degreeRotate * layers);
	rotatePoints(circle2Top, centeredPoint, degreeRotate * layers);
	drawmodel(modelTop, circle1Top, circle2Top);
}

