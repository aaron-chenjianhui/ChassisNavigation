#ifndef _POINT2D_H_
#define _POINT2D_H_

#include <iostream>

/**
 * Primitive 2D point class used as input for the LineParamEstimator.
 *
 * Author: Ziv Yaniv (zivy@cs.huji.ac.il)
 *
 * Revised: Jianhui Chen
 * Changelog (2018/7/4):
 * Add overload operator "-"
 * Add default constructor
 */

class Point2D {
public:
Point2D() : x(0), y(0)
{
}

Point2D(double px, double py) : x(px), y(py)
{
}

friend std::ostream &operator<<(std::ostream &output, const Point2D &pnt)
{
	output << pnt.x << ' ' << pnt.y;
	return output;
}

friend const Point2D operator-(const Point2D &p1, const Point2D &p2)
{
	return Point2D((p1.x - p2.x), (p1.y - p2.y));
}

friend const Point2D operator-(const Point2D &p)
{
	return Point2D(-p.x, -p.y);
}

public:
double x;
double y;
};

#endif //_POINT2D_H_
