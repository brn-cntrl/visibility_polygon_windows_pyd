#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "VisibilityPolygon.h"

namespace py = pybind11;

PYBIND11_MODULE(visibility_polygon, m) {
    m.doc() = "Visibility polygon computation library";

    py::class_<Point>(m, "Point")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def_readwrite("x", &Point::x)
        .def_readwrite("y", &Point::y)
        .def("__repr__", [](const Point& p) {
            return "Point(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
        });

    py::class_<Segment>(m, "Segment")
        .def(py::init<>())
        .def(py::init<const Point&, const Point&>())
        .def_readwrite("start", &Segment::start)
        .def_readwrite("end", &Segment::end);

    py::class_<Polygon2>(m, "Polygon2")
        .def(py::init<>())
        .def("add_vertex", &Polygon2::addVertex)
        .def_readwrite("vertices", &Polygon2::vertices);

    py::class_<SquareVisibilityData>(m, "SquareVisibilityData")
        .def(py::init<const Point&, float, float>())
        .def_readwrite("position", &SquareVisibilityData::position)
        .def_readwrite("visibility_value", &SquareVisibilityData::visibilityValue)
        .def_readwrite("scaled_radius", &SquareVisibilityData::scaledRadius);

    py::enum_<IsovistMetric>(m, "IsovistMetric")
        .value("VISIBLE_AREA", IsovistMetric::VISIBLE_AREA)
        .value("VISIBLE_PERIMETER", IsovistMetric::VISIBLE_PERIMETER)
        .value("VISIBLE_VERTICES", IsovistMetric::VISIBLE_VERTICES)
        .value("MEAN_RADIAL_DISTANCE", IsovistMetric::MEAN_RADIAL_DISTANCE)
        .export_values();

    m.def("create_circle_polygon", &createCirclePolygon, 
          "Create a circle as a polygon",
          py::arg("center"), py::arg("radius"), py::arg("segments") = 64);

    m.def("segments_intersect", &segmentsIntersect,
          "Check if two segments intersect",
          py::arg("s1"), py::arg("s2"), py::arg("intersection"));

    m.def("distance", &distance,
          "Calculate distance between two points",
          py::arg("p1"), py::arg("p2"));

    m.def("compute_visibility_polygon", 
          py::overload_cast<const Point&, const std::vector<Polygon2>&, int, int, double>(
              &computeVisibilityPolygon),
          "Compute visibility polygon from a viewpoint",
          py::arg("viewpoint"),
          py::arg("obstacles"),
          py::arg("screen_width"),
          py::arg("screen_height"),
          py::arg("ray_length"));

    m.def("compute_visibility_polygon_extended",
          [](const Point& viewpoint, const std::vector<Polygon2>& obstacles,
             int screenWidth, int screenHeight, double rayLength,
             bool visibilityRadiusSelected, double visibilityRadius,
             int circleResolution) {
              std::vector<Point> visibilityRadiusPolygon;
              auto result = computeVisibilityPolygon(
                  viewpoint, obstacles, screenWidth, screenHeight, rayLength,
                  visibilityRadiusSelected, visibilityRadius, &visibilityRadiusPolygon,
                  circleResolution, {}, nullptr);
              return py::make_tuple(result, visibilityRadiusPolygon);
          },
          "Compute visibility polygon with radius constraint, returns (visibility_polygon, radius_polygon)",
          py::arg("viewpoint"),
          py::arg("obstacles"),
          py::arg("screen_width"),
          py::arg("screen_height"),
          py::arg("ray_length"),
          py::arg("visibility_radius_selected"),
          py::arg("visibility_radius"),
          py::arg("circle_resolution") = 360);

    m.def("get_circle_line_intersection", &getCircleLineIntersection,
          "Get intersection points between a line segment and a circle",
          py::arg("line_start"), py::arg("line_end"),
          py::arg("circle_center"), py::arg("radius"));

    m.def("is_point_in_circle", &isPointInCircle,
          "Check if a point is inside a circle",
          py::arg("point"), py::arg("center"), py::arg("radius"));

    m.def("create_regular_polygon", &createRegularPolygon,
          "Create a regular polygon",
          py::arg("center_x"), py::arg("center_y"),
          py::arg("radius"), py::arg("sides"));
    
    m.def("is_point_in_polygon", &isPointInPolygon,
          "Check if a point is inside a polygon",
          py::arg("point"), py::arg("vertices"));
    
    m.def("polygons_overlap", &polygonsOverlap,
          "Check if two polygons overlap",
          py::arg("poly1"), py::arg("poly2"));

    // Clipper2 functions
    m.def("clip_circle_with_visibility_polygon", &clipCircleWithVisibilityPolygon,
          "Clip a circle with visibility polygon using Clipper2",
          py::arg("polygon"), py::arg("circle_center"),
          py::arg("radius"), py::arg("circle_segments") = 128);

    // Metric calculations
    m.def("calculate_polygon_area", &calculatePolygonArea,
          "Calculate area of a polygon",
          py::arg("vertices"));
    
    m.def("calculate_polygon_perimeter", &calculatePolygonPerimeter,
          "Calculate perimeter of a polygon",
          py::arg("vertices"));
    
    m.def("calculate_mean_radial_distance", &calculateMeanRadialDistance,
          "Calculate mean radial distance from center to polygon vertices",
          py::arg("center"), py::arg("vertices"));
}