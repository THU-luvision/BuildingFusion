#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "../mesh/Mesh.h"
#include <map>
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m" /* Yellow */
#define BLUE    "\033[34m" /* Blue */
#define PURPLE  "\033[35m" /* Purple */
#define D_GREEN "\033[36m" /* Dark Green */
    // simplify mesh with semantic and instance label.
namespace chisel{   
    //multi-resolution 3d approximations for rendering complex scenes, 1992.
    void ClusteringSimplification(MeshPtr &wait_to_simplify, float grid_len);
    //Surface Simplification Using Quadric Error Metrics, 1997.
    void QuadricSimplification(MeshPtr &wait_to_simplify, int target_triangle);

    typedef std::pair<int, int> PositionInTriangle;//triangleID, and index in triangle
    typedef std::vector<std::pair<int, int>> Reference;//For each vertex, reference is used to find contained triangles
    struct QuadricHelper
    {
        std::vector<bool> t_dirty;
        std::vector<bool> t_deleted;
        Vec3List *points_ptr;
        Vec3List *normals_ptr;
        Vec3List *colors_ptr;
        Point3List *triangles_ptr;
        std::vector<long> *labels_ptr;
        Mat4List QMatrix;
        std::vector<std::vector<float>> error;
        std::vector<Reference> references;
        std::vector<bool> is_border;
        Vec3List normals;//plane normal
        int deleted_triangle = 0;
    };

    struct ClusteringHelper
    {
        std::vector<bool> t_deleted;
        Vec3List *points_ptr;
        Vec3List *normals_ptr;
        Vec3List *colors_ptr;
        Point3List *triangles_ptr;   
        std::vector<long> *labels_ptr;   
        std::vector<Reference> references;  
        float grid_len;
        std::unordered_map<Eigen::Vector3i, int ,ChunkHasher> grid_map;
        
    };
    Eigen::Vector3i GetGridIndex(const Eigen::Vector3f &points, float grid_size);
    bool Flipped(QuadricHelper &helper, int p1, int p2, Eigen::Vector3f v, std::vector<bool> &deleted);
    void UpdateTriangles(QuadricHelper &helper,int p1, int p2, std::vector<bool> &deleted);
    void UpdateMesh(QuadricHelper &helper);
    void UpdateMesh(ClusteringHelper &helper);
    void ComputeVertexNormals(QuadricHelper &helper);
    void ComputeVertexNormals(ClusteringHelper &helper);
    void CompactMesh(QuadricHelper &helper);
    void CompactMesh(ClusteringHelper &helper);
    void InitializeHelper(MeshPtr &wait_to_simplify, QuadricHelper &helper);
    void InitializeHelper(MeshPtr &wait_to_simplify, ClusteringHelper &helper);
    void ComputeError(QuadricHelper &helper);
    void CheckIsBorder(const Point3List &triangles, const std::vector<Reference> &references, std::vector<bool> &is_border);
    float ComputeError(QuadricHelper &helper, int i, int j, Eigen::Vector3f &new_p);
    void UpdateReferences(const Point3List &triangles, std::vector<Reference> &references);
    void ComputeNormalsAndQMatrix(const Point3List &triangles, const Vec3List &points,
        std::vector<Reference> &references, Vec3List &normals ,Mat4List &QMatrix);
}