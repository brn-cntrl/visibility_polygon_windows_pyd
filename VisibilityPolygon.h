#pragma once
#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif

#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <iostream>
#include <limits>
#include "clipper2/clipper.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Point {
	double x, y;
	Point() : x(0), y(0) {}
	Point(double x, double y) : x(x), y(y) {}
};

struct Segment {
	Point start, end;
	Segment() {}
	Segment(const Point& start, const Point& end) : start(start), end(end) {}
};

struct Polygon2 {
	std::vector<Point> vertices;
	void addVertex(double x, double y) {
		vertices.push_back(Point(x, y));
	}
};

struct SquareVisibilityData {
	Point position;
	float visibilityValue;
	float scaledRadius;  // The actual radius to use for visibility calculations
	
	SquareVisibilityData(const Point& pos, float value, float radius)
		: position(pos), visibilityValue(value), scaledRadius(radius) {}
};

std::vector<Point> createCirclePolygon(const Point& center, double radius, int segments = 64);

bool segmentsIntersect(const Segment& s1, const Segment& s2, Point& intersection);
double distance(const Point& p1, const Point& p2);

std::vector<Point> computeVisibilityPolygon(const Point& viewpoint,
										   const std::vector<Polygon2>& obstacles,
										   int screenWidth, int screenHeight,
										   double rayLength,
										   bool visibilityRadiusSelected,
										   double visibilityRadius,
										   std::vector<Point>* visibilityRadiusPolygon,
										   int circleResolution,
										   const std::vector<SquareVisibilityData>& squareVisibility = {},
										   std::vector<std::vector<Point>>* squareVisibilityPolygons = nullptr);

// Backward compatibility overload
std::vector<Point> computeVisibilityPolygon(const Point& viewpoint,
										   const std::vector<Polygon2>& obstacles,
										   int screenWidth, int screenHeight,
										   double rayLength);

std::vector<Point> getCircleLineIntersection(const Point& lineStart,
										   const Point& lineEnd,
										   const Point& circleCenter,
										   double radius);

bool isPointInCircle(const Point& point, const Point& center, double radius);

Polygon2 createRegularPolygon(double centerX, double centerY, double radius, int sides);
bool isPointInPolygon(const Point& point, const std::vector<Point>& vertices);
bool polygonsOverlap(const Polygon2& poly1, const Polygon2& poly2);

enum IsovistMetric {
	VISIBLE_AREA,           // Total visible area
	VISIBLE_PERIMETER,      // Perimeter of visibility polygon
	VISIBLE_VERTICES,       // Number of vertices in visibility polygon
	MEAN_RADIAL_DISTANCE    // Average distance to visibility boundary
};

std::vector<Point> clipCircleWithVisibilityPolygon(const std::vector<Point>& polygon,
												   const Point& circleCenter,
												   double radius,
												   int circleSegments = 128);

float calculatePolygonArea(const std::vector<Point>& vertices);
float calculatePolygonPerimeter(const std::vector<Point>& vertices);
float calculateMeanRadialDistance(const Point& center, const std::vector<Point>& vertices);

