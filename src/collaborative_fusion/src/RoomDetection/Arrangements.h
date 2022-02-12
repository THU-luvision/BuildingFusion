#ifndef ARRANGEMENT_OF_LINES_H
#define ARRANGEMENT_OF_LINES_H

#include "DCEL.h"
#include <memory>
//A demo for arrangments of lines: https://linearrangement.stackblitz.io/
void ComputeIntersect(const std::vector<Line> &lines, 
    Point2fList &intersect_points);
std::shared_ptr<DCEL> CreateBoundingBoxDcel(const Point2fList &points);
#endif