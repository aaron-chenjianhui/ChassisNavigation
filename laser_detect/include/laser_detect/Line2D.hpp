#ifndef _LINE2D_HPP
#define _LINE2D_HPP

#include "Point2D.h"
#include "MatrixCal.h"

class Line2D {
public:
typedef std::vector<Point2D> PointsT;

public:

Line2D() : m_ax(0), m_ay(0), m_nx(0), m_ny(1)
{
}

Line2D(double ax, double ay, double nx, double ny)
	: m_ax(ax), m_ay(ay), m_nx(nx), m_ny(ny)
{
}
~Line2D()
{
}

Point2D Normal()
{
	return Point2D(m_nx, m_ny);
}

Point2D Normal() const
{
	return Point2D(m_nx, m_ny);
}

double GetYVal(double x)
{
	return (m_ax * m_nx + m_ay * m_ny - x * m_nx) / m_ny;
}

Point2D PointInLine()
{
	return Point2D(m_ax, m_ay);
}

Point2D PointInLine() const
{
	return Point2D(m_ax, m_ay);
}

static double DotProduct(const Point2D& p1, const Point2D& p2)
{
	return p1.x * p2.x + p1.y * p2.y;
}

static Point2D CrossPoint(const Line2D& line1, const Line2D& line2)
{
	mat2x2 a;

	a(0, 0) = line1.nx;
	a(0, 1) = line1.ny;
	a(1, 0) = line2.nx;
	a(1, 1) = line2.ny;
	mat2x1 b;
	b(0) = line1.ax * line1.nx + line1.ay * line1.ny;
	b(1) = line2.ax * line2.nx + line2.ay * line2.ny;

	mat2x1 p;
	p = a.inverse() * b;

	return Point2D(p(0), p(1));
}

// Find the reference location of point
void PointLoc()
{
}

// For Line2D display
PointsT GenPoints(double x_start, double x_end, int num = 500)
{
	PointsT points;
	Point2D point;
	double x_incre = (x_end - x_start) / num;

	for (int i = 0; i < num; ++i) {
		point.x = x_start + x_incre * i;
		point.y = GetYVal(point.x);

		points.push_back(point);
	}

	return points;
}

double m_ax;
double m_ay;
double m_nx;
double m_ny;
};


#endif
