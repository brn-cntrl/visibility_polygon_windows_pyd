#ifdef _WIN32
#define _USE_MATH_DEFINES
#endif

#include "VisibilityPolygon.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "clipper2/clipper.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const int MIN_SIDES = 3;                // Minimum sides for polygons
const int MAX_SIDES = 16;               // Maximum sides for polygons
const int MIN_RADIUS = 20;              // Minimum radius for polygons
const int MAX_RADIUS = 50;              // Maximum radius for polygons
const int RAY_LENGTH = 1000;            // Length of rays for visibility calculation
const int NUM_RAYS = 360;               // Number of rays to cast (one per degree)
const int MAX_PLACEMENT_ATTEMPTS = 100; // Maximum attempts to place a non-overlapping polygon

bool segmentsIntersect(const Segment& s1, const Segment& s2, Point& intersection) {
	double x1 = s1.start.x, y1 = s1.start.y;
	double x2 = s1.end.x, y2 = s1.end.y;
	double x3 = s2.start.x, y3 = s2.start.y;
	double x4 = s2.end.x, y4 = s2.end.y;
	
	double den = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
	
	// If den is zero, the segments are parallel or collinear
	if (den == 0) {
		return false;
	}
	
	// Calculate the numerators
	double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / den;
	double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / den;
	
	// If ua and ub are both between 0 and 1, segments intersect
	if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
		// Calculate the intersection point
		intersection.x = x1 + ua * (x2 - x1);
		intersection.y = y1 + ua * (y2 - y1);
		return true;
	}
	
	return false;
}

double distance(const Point& p1, const Point& p2) {
	double dx = p2.x - p1.x;
	double dy = p2.y - p1.y;
	return std::sqrt(dx * dx + dy * dy);
}

std::vector<Point> computeVisibilityPolygon(const Point& viewpoint,
										   const std::vector<Polygon2>& obstacles,
										   int screenWidth, int screenHeight,
										   double rayLength,
										   bool visibilityRadiusSelected,
										   double visibilityRadius,
										   std::vector<Point>* visibilityRadiusPolygon,
										   int circleResolution,
										   const std::vector<SquareVisibilityData>& squareVisibility,
										   std::vector<std::vector<Point>>* squareVisibilityPolygons) {
	
	std::vector<Point> visibilityPolygon;
	std::vector<Point> radiusPolygon;
	
	if (visibilityRadiusPolygon) {
		visibilityRadiusPolygon->clear();
	}
	
	// Clear square visibility polygons but don't use them
	if (squareVisibilityPolygons) {
		squareVisibilityPolygons->clear();
	}
	
	std::vector<Segment> allSegments;
	
	for (const auto& obstacle : obstacles) {
		for (size_t i = 0; i < obstacle.vertices.size(); ++i) {
			size_t j = (i + 1) % obstacle.vertices.size();
			allSegments.push_back(Segment(obstacle.vertices[i], obstacle.vertices[j]));
		}
	}
	
	allSegments.push_back(Segment(Point(0, 0), Point(screenWidth, 0)));
	allSegments.push_back(Segment(Point(screenWidth, 0), Point(screenWidth, screenHeight)));
	allSegments.push_back(Segment(Point(screenWidth, screenHeight), Point(0, screenHeight)));
	allSegments.push_back(Segment(Point(0, screenHeight), Point(0, 0)));
	
	std::vector<double> angles;
	
	for (const auto& obstacle : obstacles) {
		for (const auto& vertex : obstacle.vertices) {
			double angle = std::atan2(vertex.y - viewpoint.y, vertex.x - viewpoint.x);
			
			angles.push_back(angle);
			
			const double epsilon = 0.0001;
			angles.push_back(angle - epsilon);
			angles.push_back(angle + epsilon);
		}
	}
	
	std::vector<Point> corners = {
		Point(0, 0), Point(screenWidth, 0),
		Point(screenWidth, screenHeight), Point(0, screenHeight)
	};
	
	for (const auto& corner : corners) {
		double angle = std::atan2(corner.y - viewpoint.y, corner.x - viewpoint.x);
		angles.push_back(angle);
		const double epsilon = 0.0001;
		angles.push_back(angle - epsilon);
		angles.push_back(angle + epsilon);
	}
	
	if (visibilityRadiusSelected) {
		for (int i = 0; i < circleResolution; i++) {
			double angle = 2.0 * M_PI * i / circleResolution;
			angles.push_back(angle);
		}
	}
	
	std::sort(angles.begin(), angles.end());
	angles.erase(std::unique(angles.begin(), angles.end(),
							[](double a, double b) { return std::abs(a - b) < 0.00001; }),
				 angles.end());
	
	for (double angle : angles) {
		double dx = std::cos(angle);
		double dy = std::sin(angle);
		
		Point rayEnd(viewpoint.x + rayLength * dx, viewpoint.y + rayLength * dy);
		Segment ray(viewpoint, rayEnd);
		
		Point closestIntersection;
		double closestDistance = std::numeric_limits<double>::max();
		bool found = false;
		
		for (const auto& segment : allSegments) {
			Point intersection;
			if (segmentsIntersect(ray, segment, intersection)) {
				double dist = distance(viewpoint, intersection);
				if (dist < closestDistance && dist > 0.001) {
					closestDistance = dist;
					closestIntersection = intersection;
					found = true;
				}
			}
		}
		
		if (found) {
			visibilityPolygon.push_back(closestIntersection);
			
			if (visibilityRadiusSelected && visibilityRadiusPolygon != nullptr) {
				if (closestDistance <= visibilityRadius) {
					radiusPolygon.push_back(closestIntersection);
				} else {
					Point circlePoint(viewpoint.x + visibilityRadius * dx,
									viewpoint.y + visibilityRadius * dy);
					radiusPolygon.push_back(circlePoint);
				}
			}
		}
	}
	
	auto sortByAngle = [&viewpoint](const Point& a, const Point& b) {
		double angleA = std::atan2(a.y - viewpoint.y, a.x - viewpoint.x);
		double angleB = std::atan2(b.y - viewpoint.y, b.x - viewpoint.x);
		return angleA < angleB;
	};
	
	std::sort(visibilityPolygon.begin(), visibilityPolygon.end(), sortByAngle);
	
	if (visibilityRadiusSelected && visibilityRadiusPolygon != nullptr) {
		std::sort(radiusPolygon.begin(), radiusPolygon.end(), sortByAngle);
		*visibilityRadiusPolygon = radiusPolygon;
	}
	
	return visibilityPolygon;
}


std::vector<Point> computeVisibilityPolygon(const Point& viewpoint,
										   const std::vector<Polygon2>& obstacles,
										   int screenWidth, int screenHeight,
										   double rayLength) {
	
	return computeVisibilityPolygon(viewpoint, obstacles, screenWidth, screenHeight,
								  rayLength, false, 0.0, nullptr, 360, {}, nullptr);
}

Polygon2 createRegularPolygon(double centerX, double centerY, double radius, int sides) {
	Polygon2 polygon;
	for (int i = 0; i < sides; ++i) {
		double angle = 2.0 * M_PI * i / sides;
		double x = centerX + radius * std::cos(angle);
		double y = centerY + radius * std::sin(angle);
		polygon.addVertex(x, y);
	}
	return polygon;
}

std::vector<Point> createCirclePolygon(const Point& center, double radius, int segments) {
	std::vector<Point> circle;
	for (int i = 0; i < segments; i++) {
		double angle = 2.0 * M_PI * i / segments;
		double x = center.x + radius * std::cos(angle);
		double y = center.y + radius * std::sin(angle);
		circle.push_back(Point(x, y));
	}
	return circle;
}

bool isPointInPolygon(const Point& point, const std::vector<Point>& vertices) {
	if (vertices.size() < 3) return false;
	
	bool inside = false;
	size_t j = vertices.size() - 1;
	
	for (size_t i = 0; i < vertices.size(); i++) {
		if (((vertices[i].y > point.y) != (vertices[j].y > point.y)) &&
			(point.x < (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) /
			 (vertices[j].y - vertices[i].y) + vertices[i].x)) {
			inside = !inside;
		}
		j = i;
	}
	
	return inside;
}

bool polygonsOverlap(const Polygon2& poly1, const Polygon2& poly2) {
	Point centroid1 = {0, 0};
	Point centroid2 = {0, 0};
	
	for (const auto& v : poly1.vertices) {
		centroid1.x += v.x;
		centroid1.y += v.y;
	}
	centroid1.x /= poly1.vertices.size();
	centroid1.y /= poly1.vertices.size();
	
	for (const auto& v : poly2.vertices) {
		centroid2.x += v.x;
		centroid2.y += v.y;
	}
	centroid2.x /= poly2.vertices.size();
	centroid2.y /= poly2.vertices.size();
	
	double radius1 = 0, radius2 = 0;
	for (const auto& v : poly1.vertices) {
		double dist = distance(centroid1, v);
		if (dist > radius1) radius1 = dist;
	}
	for (const auto& v : poly2.vertices) {
		double dist = distance(centroid2, v);
		if (dist > radius2) radius2 = dist;
	}
	
	double centerDistance = distance(centroid1, centroid2);
	if (centerDistance > radius1 + radius2) {
		return false; // Definitely not overlapping
	}
	
	for (const auto& v : poly1.vertices) {
		if (isPointInPolygon(v, poly2.vertices)) {
			return true;
		}
	}
	
	for (const auto& v : poly2.vertices) {
		if (isPointInPolygon(v, poly1.vertices)) {
			return true;
		}
	}
	
	for (size_t i = 0; i < poly1.vertices.size(); i++) {
		size_t i2 = (i + 1) % poly1.vertices.size();
		Segment seg1(poly1.vertices[i], poly1.vertices[i2]);
		
		for (size_t j = 0; j < poly2.vertices.size(); j++) {
			size_t j2 = (j + 1) % poly2.vertices.size();
			Segment seg2(poly2.vertices[j], poly2.vertices[j2]);
			
			Point intersection;
			if (segmentsIntersect(seg1, seg2, intersection)) {
				return true;
			}
		}
	}
	
	return false;
}

std::vector<Point> getCircleLineIntersection(const Point& lineStart,
										   const Point& lineEnd,
										   const Point& circleCenter,
										   double radius) {
	std::vector<Point> intersections;
	
	double dx = lineStart.x - circleCenter.x;
	double dy = lineStart.y - circleCenter.y;
	
	double vx = lineEnd.x - lineStart.x;
	double vy = lineEnd.y - lineStart.y;
	
	double a = vx * vx + vy * vy;
	double b = 2 * (dx * vx + dy * vy);
	double c = dx * dx + dy * dy - radius * radius;
	
	double discriminant = b * b - 4 * a * c;
	
	if (discriminant < 0) {
		return intersections; // No intersection
	}
	
	if (a == 0) {
		return intersections; // Line has zero length
	}
	
	double sqrtDisc = std::sqrt(discriminant);
	double t1 = (-b - sqrtDisc) / (2 * a);
	double t2 = (-b + sqrtDisc) / (2 * a);
	
	if (t1 >= 0 && t1 <= 1) {
		Point intersection;
		intersection.x = lineStart.x + t1 * vx;
		intersection.y = lineStart.y + t1 * vy;
		intersections.push_back(intersection);
	}
	
	if (t2 >= 0 && t2 <= 1 && std::abs(t2 - t1) > 1e-10) {
		Point intersection;
		intersection.x = lineStart.x + t2 * vx;
		intersection.y = lineStart.y + t2 * vy;
		intersections.push_back(intersection);
	}
	
	return intersections;
}

bool isPointInCircle(const Point& point, const Point& center, double radius) {
	return distance(point, center) <= radius;
}

float calculatePolygonArea(const std::vector<Point>& vertices) {
	if (vertices.size() < 3) return 0.0f;
	
	float area = 0.0f;
	int n = vertices.size();
	
	for (int i = 0; i < n; i++) {
		int j = (i + 1) % n;
		area += vertices[i].x * vertices[j].y;
		area -= vertices[j].x * vertices[i].y;
	}
	
	return std::abs(area) / 2.0f;
}

float calculatePolygonPerimeter(const std::vector<Point>& vertices) {
	if (vertices.size() < 2) return 0.0f;
	
	float perimeter = 0.0f;
	int n = vertices.size();
	
	for (int i = 0; i < n; i++) {
		int j = (i + 1) % n;
		perimeter += distance(vertices[i], vertices[j]);
	}
	
	return perimeter;
}

float calculateMeanRadialDistance(const Point& center, const std::vector<Point>& vertices) {
	if (vertices.empty()) return 0.0f;
	
	float totalDistance = 0.0f;
	for (const auto& vertex : vertices) {
		totalDistance += distance(center, vertex);
	}
	
	return totalDistance / vertices.size();
}

// Convert from your Point format to Clipper2 PathD
Clipper2Lib::PathD convertToClipperPath(const std::vector<Point>& points) {
	Clipper2Lib::PathD path;
	path.reserve(points.size());
	for (const auto& pt : points) {
		path.emplace_back(pt.x, pt.y);
	}
	return path;
}

// Convert from Clipper2 PathD to your Point format
std::vector<Point> convertFromClipperPath(const Clipper2Lib::PathD& path) {
	std::vector<Point> points;
	points.reserve(path.size());
	for (const auto& pt : path) {
		points.emplace_back(pt.x, pt.y);
	}
	return points;
}

// Create a circle as a Clipper2 PathD
Clipper2Lib::PathD createClipperCircle(const Point& center, double radius, int segments) {
	Clipper2Lib::PathD circle;
	circle.reserve(segments);
	
	for (int i = 0; i < segments; i++) {
		double angle = 2.0 * M_PI * i / segments;
		double x = center.x + radius * std::cos(angle);
		double y = center.y + radius * std::sin(angle);
		circle.emplace_back(x, y);
	}
	
	return circle;
}

std::vector<Point> clipCircleWithVisibilityPolygon(const std::vector<Point>& polygon,
												   const Point& circleCenter,
												   double radius,
												   int circleSegments) {
	if (polygon.empty() || radius <= 0) {
		return std::vector<Point>();
	}
	
	try {
		Clipper2Lib::PathD subjectPath = convertToClipperPath(polygon);
		Clipper2Lib::PathD clipPath = createClipperCircle(circleCenter, radius, circleSegments);
		Clipper2Lib::ClipperD clipper;

		clipper.AddSubject({subjectPath});
		clipper.AddClip({clipPath});
		Clipper2Lib::PathsD solution;

		if (!clipper.Execute(Clipper2Lib::ClipType::Intersection,
						   Clipper2Lib::FillRule::NonZero, solution)) {
			std::cout << "Clipper execution failed" << std::endl;
			return std::vector<Point>();
		}
		
		if (solution.empty()) {
            std::cout << "No intersection found" << std::endl;
			return std::vector<Point>();
		}
		
		if (solution.size() == 1) {
            std::cout << "Single intersection polygon found" << std::endl;
			return convertFromClipperPath(solution[0]);
		} else {
			size_t largestIndex = 0;
			double largestArea = 0;
			for (size_t i = 0; i < solution.size(); i++) {
				double area = std::abs(Clipper2Lib::Area(solution[i]));
				if (area > largestArea) {
					largestArea = area;
					largestIndex = i;
				}
			}
            std::cout << "Multiple intersection polygons found, returning largest with area: " << largestArea << std::endl;
			return convertFromClipperPath(solution[largestIndex]);
		}
		
	} catch (const std::exception& e) {
        std::cout << "Exception during clipping: " << e.what() << std::endl;
		return std::vector<Point>();
	}
}
