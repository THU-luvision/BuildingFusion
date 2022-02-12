#ifndef SIMPLIFY_SIMD
#define SIMPLIFY_SIME
#include <open_chisel/mesh/Mesh.h>

//These are used for SIMD acceleration
#include <xmmintrin.h>
#include <smmintrin.h>
#include <immintrin.h>
namespace chisel
{
    class SimplifierSIMD
    {
        public:
        SimplifierSIMD()=default;

        inline size_t load_mesh_from_compact(MeshPtr &mesh);
        inline size_t save_mesh_to_compact(MeshPtr &m);	
        void compact_to_no(MeshPtr &m);
        void write_obj(const char* filename);
        void computeNormal();
        inline size_t load_mesh(chisel::MeshPtr &mesh);
        inline size_t save_mesh(MeshPtr &mesh);
        void simplify_mesh_from_CHISEL_compact(MeshPtr &m,float compact_ratio = 0.5);	
        double vertex_error(SymetricMatrix q, double x, double y, double z);
        double calculate_error(int id_v1, int id_v2, vec3f &p_result);
        bool flipped(vec3f p,int i0,int i1,Vertex &v0,Vertex &v1,std::vector<int> &deleted);
        void update_triangles(int i0,Vertex &v,std::vector<int> &deleted,int &deleted_triangles);
        void update_mesh(int iteration);
        void compact_mesh();	
        void reset();	
        void simplify_mesh(int target_count, double agressiveness=5, bool verbose=false);
        std::vector<_Color> *colors;
        std::vector<Triangle> *triangles;
        std::vector<Vertex> *vertices;
        std::vector<Normal> *normals;
        std::vector<> refs;
    };
}

#endif