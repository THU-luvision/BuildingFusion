#ifndef RPLY_READER_H
#define RPLY_READER_H
#include <vector>
#include <Eigen/Core>
#include <open_chisel/mesh/Mesh.h>
#include <open_chisel/mesh/PointCloud.h>
#include "../Geometry/Geometry.h"
#include <open_chisel/ForLabel.h>
bool ReadPLY(const std::string &filename, Point3fList &points, Point3fList &normals, 
    Point3fList &colors);
bool ReadPLY(const std::string &filename, Point3fList &points, Point3fList &normals, 
    Point3fList &colors, Point3iList &triangles);
bool WritePLY(const std::string &filename, const Point3fList&points, 
    const Point3fList &normals, const Point3fList &colors);
bool WritePLY(const std::string &filename, const Point3fList&points, 
    const Point3fList &normals, const Point3fList &colors,
    const Point3iList &triangles);
bool WritePLY(const std::string &filename, Point3fList &points, Point3fList &normals, 
    Point3fList &colors, std::vector<long> &labels);
bool ReadPLYToChiselMesh(const std::string &filename, chisel::MeshPtr &mesh);
bool WritePLYFromChiselMesh(const std::string &filename, chisel::MeshPtr &mesh);
bool ReadPLYToChiselPcd(const std::string &filename, chisel::PcdPtr &pcd);
bool WritePLYFromChiselPcd(const std::string &filename, chisel::PcdPtr &pcd);

bool WritePLYFromChiselPcdWithLabel(const std::string &filename, chisel::PcdPtr &pcd);
#endif