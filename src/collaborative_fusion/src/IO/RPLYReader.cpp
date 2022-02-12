
#include "./rply/rply.h"
#include <iostream>
#include <fstream>
#include "RPLYReader.h"


//cb means callback
//create by Guoqing Zhang 2019.12.18
typedef Point3fList VertexSet;
typedef Point3fList NormalSet;
typedef Point3iList TriangleSet;
typedef Point3fList ColorSet;

struct PlyState
{
    int vertex_index;
    int vertex_num;
    int normal_index;
    int normal_num;
    int color_index;
    int color_num;
    int triangle_index;
    int triangle_num;
    VertexSet *vertex_ptr;
    NormalSet *normal_ptr;
    ColorSet *color_ptr;
    TriangleSet *triangle_ptr;
};

int vertex_cb(p_ply_argument argument) {
    long index;
    PlyState *plystate_ptr;
    ply_get_argument_user_data(argument, reinterpret_cast<void **>(&plystate_ptr), &index);
    if(plystate_ptr->vertex_index >= plystate_ptr->vertex_num)
    return 0;
    float value = ply_get_argument_value(argument);
    (*plystate_ptr->vertex_ptr)[plystate_ptr->vertex_index](index) = value;
    if(index == 2)
    plystate_ptr->vertex_index++;    
    return 1;
}
int normal_cb(p_ply_argument argument) 
{
    long index;
    PlyState *plystate_ptr;
    ply_get_argument_user_data(argument, reinterpret_cast<void **>(&plystate_ptr), &index);
    if(plystate_ptr->normal_index >= plystate_ptr->normal_num)
    return 0;
    float value = ply_get_argument_value(argument);
    (*plystate_ptr->normal_ptr)[plystate_ptr->normal_index](index) = value;
    if(index == 2)
    plystate_ptr->normal_index++;    
    return 1;
}

int color_cb(p_ply_argument argument)
{
    long index;
    PlyState *plystate_ptr;
    ply_get_argument_user_data(argument, reinterpret_cast<void **>(&plystate_ptr), &index);
    if(plystate_ptr->color_index >= plystate_ptr->color_num)
    return 0;
    float value = ply_get_argument_value(argument);
    (*plystate_ptr->color_ptr)[plystate_ptr->color_index](index) = value/255.0;
    if(index == 2)
    plystate_ptr->color_index++;    
    return 1;
}



int triangle_cb(p_ply_argument argument) 
{
PlyState *plystate_ptr;
long dummy, length, index;
ply_get_argument_user_data(argument, reinterpret_cast<void **>(&plystate_ptr),
                            &dummy);
float value = ply_get_argument_value(argument);
if (plystate_ptr->triangle_index >= plystate_ptr->triangle_num) {
    return 0;
}

ply_get_argument_property(argument, NULL, &length, &index);
if (index == -1) ;
else
    (*plystate_ptr->triangle_ptr)[plystate_ptr->triangle_index](index) = value;    
if (index == length-1) {
    plystate_ptr->triangle_index++;
}
return 1;
}

bool ReadPLY(const std::string &filename, Point3fList &points, Point3fList &normals, 
    Point3fList &colors, Point3iList &triangles)
{
    p_ply ply_file = ply_open(filename.c_str(), NULL, 0, NULL);
    if (!ply_file) {
        std::cout <<RED << "[PLYReader]::[ERROR]::Cannot open file "<<filename<<RESET<<std::endl;
        return false;
    }
    if (!ply_read_header(ply_file)) {
        std::cout <<RED << "[PLYReader]::[ERROR]::Cannot Parse file header "<<RESET<<std::endl;
        ply_close(ply_file);
        return false;
    }

    PlyState state;


    state.vertex_num = ply_set_read_cb(ply_file, "vertex", "x",
                                    vertex_cb, &state, 0);
    ply_set_read_cb(ply_file, "vertex", "y", vertex_cb, &state, 1);
    ply_set_read_cb(ply_file, "vertex", "z", vertex_cb, &state, 2);

    state.normal_num = ply_set_read_cb(ply_file, "vertex", "nx",
                                    normal_cb, &state, 0);
    ply_set_read_cb(ply_file, "vertex", "ny", normal_cb, &state, 1);
    ply_set_read_cb(ply_file, "vertex", "nz", normal_cb, &state, 2);

    state.color_num = ply_set_read_cb(ply_file, "vertex", "red",
                                    color_cb, &state, 0);
    ply_set_read_cb(ply_file, "vertex", "green", color_cb, &state, 1);
    ply_set_read_cb(ply_file, "vertex", "blue", color_cb, &state, 2);

    if (state.vertex_num <= 0) {
        std::cout <<RED << "[PLYReader]::[ERROR]::vertex_num < 0"<<RESET<<std::endl;
        ply_close(ply_file);
        return false;
    }

    state.triangle_num = ply_set_read_cb(ply_file, "face", "vertex_indices",
                                    triangle_cb, &state, 0);
    if (state.triangle_num == 0) {
        state.triangle_num = ply_set_read_cb(ply_file, "face", "vertex_index",
                                        triangle_cb, &state, 0);
    }

    state.vertex_index = 0;
    state.normal_index = 0;
    state.color_index = 0; 
    state.triangle_index = 0;

    points.resize(state.vertex_num);
    normals.resize(state.normal_num);
    triangles.resize(state.triangle_num);
    colors.resize(state.color_num);

    state.vertex_ptr = &points;
    state.normal_ptr = &normals; 
    state.triangle_ptr = &triangles;
    state.color_ptr = &colors;
    if (!ply_read(ply_file)) 
    {
        std::cout <<RED << "[PLYReader]::[ERROR]::failed to read "<<filename<<RESET<<std::endl;
        ply_close(ply_file);
        return false;
    }
    std::cout <<BLUE<<"[PLYReader]::[INFO]::"<<"face: "<<state.triangle_num<<" vertex: "<<state.vertex_num<<RESET<<std::endl;
    ply_close(ply_file);
}

bool ReadPLY(const std::string &filename, Point3fList &points, Point3fList &normals, 
    Point3fList &colors)
{
    p_ply ply_file = ply_open(filename.c_str(), NULL, 0, NULL);
    if (!ply_file) {
        std::cout <<RED << "[PLYReader]::[ERROR]::Cannot open file "<<filename<<RESET<<std::endl;
        return false;
    }
    if (!ply_read_header(ply_file)) {
        std::cout <<RED << "[PLYReader]::[ERROR]::Cannot Parse file header "<<RESET<<std::endl;
        ply_close(ply_file);
        return false;
    }

    PlyState state;


    state.vertex_num = ply_set_read_cb(ply_file, "vertex", "x",
                                    vertex_cb, &state, 0);
    ply_set_read_cb(ply_file, "vertex", "y", vertex_cb, &state, 1);
    ply_set_read_cb(ply_file, "vertex", "z", vertex_cb, &state, 2);

    state.normal_num = ply_set_read_cb(ply_file, "vertex", "nx",
                                    normal_cb, &state, 0);
    ply_set_read_cb(ply_file, "vertex", "ny", normal_cb, &state, 1);
    ply_set_read_cb(ply_file, "vertex", "nz", normal_cb, &state, 2);

    state.color_num = ply_set_read_cb(ply_file, "vertex", "red",
                                    color_cb, &state, 0);
    ply_set_read_cb(ply_file, "vertex", "green", color_cb, &state, 1);
    ply_set_read_cb(ply_file, "vertex", "blue", color_cb, &state, 2);

    if (state.vertex_num <= 0) {
        std::cout <<RED << "[PLYReader]::[ERROR]::vertex_num < 0"<<RESET<<std::endl;
        ply_close(ply_file);
        return false;
    }

    state.vertex_index = 0;
    state.normal_index = 0;
    state.color_index = 0; 

    points.resize(state.vertex_num);
    normals.resize(state.normal_num);
    colors.resize(state.color_num);

    state.vertex_ptr = &points;
    state.normal_ptr = &normals; 
    state.color_ptr = &colors;
    if (!ply_read(ply_file)) 
    {
        std::cout <<RED << "[PLYReader]::[ERROR]::failed to read "<<filename<<RESET<<std::endl;
        ply_close(ply_file);
        return false;
    }
    std::cout <<BLUE<<"[PLYReader]::[INFO]::vertex: "<<state.vertex_num<<RESET<<std::endl;
    ply_close(ply_file);
}
bool WritePLY(const std::string &filename, const Point3fList&points, 
    const Point3fList &normals, const Point3fList &colors)
{
    std::ofstream stream(filename.c_str());
    if (!stream)
    {
        std::cout<<RED << "[PLYWriter]::[ERROR] Cannot open file "<<filename<<RESET<<std::endl;
        return false;
    }
    size_t numPoints = points.size();
    bool has_normals = normals.size()>0 && normals.size() == points.size();
    bool has_colors = colors.size() > 0 && colors.size() == points.size();
    stream << "ply" << std::endl;
    stream << "format ascii 1.0" << std::endl;
    stream << "element vertex " << numPoints << std::endl;
    stream << "property float x" << std::endl;
    stream << "property float y" << std::endl;
    stream << "property float z" << std::endl;
    if(has_normals)
    {
        stream << "property float nx"<<std::endl;
        stream << "property float ny"<<std::endl;
        stream << "property float nz"<<std::endl;
    }
    if (has_colors)
    {
        stream << "property uchar red" << std::endl;
        stream << "property uchar green" << std::endl;
        stream << "property uchar blue" << std::endl;
    }
    stream << "end_header" << std::endl;
    for (int i = 0;i!=numPoints;++i)
    {
        //Eigen::Vector3f vertd = R * vert.cast<float>();
        stream << points[i](0) << " " << points[i](1) << " " << points[i](2);
        if (has_normals)
        {
            stream << " " << normals[i](0) << " " << normals[i](1) << " " << normals[i](2);
        }
        if (has_colors)
        {
            
            int r = static_cast<int>(colors[i](0) * 255.0f);
            int g = static_cast<int>(colors[i](1) * 255.0f);
            int b = static_cast<int>(colors[i](2) * 255.0f);

            stream << " " << r << " " << g << " " << b;
        }

        stream << std::endl;
    }
    return true;
}
bool WritePLY(const std::string &filename, const Point3fList&points, 
    const Point3fList &normals, const Point3fList &colors,
    const Point3iList &triangles)
{
        size_t numPoints = points.size();
        bool has_normals = normals.size()>0 && normals.size() == points.size();
        bool has_colors = colors.size() > 0 && colors.size() == points.size();
        //write ply with instance and semantic label
        if(points.size() == 0) 
        {
            return true;
        }

        p_ply ply_file = ply_create(filename.c_str(), PLY_LITTLE_ENDIAN,
                                    NULL, 0, NULL);
        if(!ply_file) {
            std::cout<<RED << "[PLYWriter]::[ERROR] Cannot open file "<<filename<<RESET<<std::endl;
            return false;
        }
        ply_add_comment(ply_file, "Created by FCFusion");
        ply_add_element(ply_file, "vertex",
                        static_cast<long>(points.size()));
        ply_add_property(ply_file, "x", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "y", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "z", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        if(has_normals)
        {
            ply_add_property(ply_file, "nx", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
            ply_add_property(ply_file, "ny", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
            ply_add_property(ply_file, "nz", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        }
        if(has_colors) 
        {
            ply_add_property(ply_file, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
            ply_add_property(ply_file, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
            ply_add_property(ply_file, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
        }
        ply_add_element(ply_file, "face",
                        static_cast<long>(triangles.size()));
        ply_add_property(ply_file, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_UINT);
        /*
        if(labels.size() > 0)
        {
            ply_add_property(ply_file, "semantic", PLY_INT32, PLY_INT32, PLY_INT32);
            ply_add_property(ply_file, "instance", PLY_INT32, PLY_INT32, PLY_INT32);
        }*/

        if(!ply_write_header(ply_file))
        {
            std::cout<<"Unable to write header."<<std::endl;
            return false;
        }
        for (size_t i = 0; i < points.size(); i++)
        {
            const Eigen::Vector3f &point = points[i];
            ply_write(ply_file, point(0));
            ply_write(ply_file, point(1));
            ply_write(ply_file, point(2));
            if(has_normals) 
            {
                const Eigen::Vector3f &normal = normals[i];
                ply_write(ply_file, normal(0));
                ply_write(ply_file, normal(1));
                ply_write(ply_file, normal(2));
            }
            if(has_colors) 
            {
                const Eigen::Vector3f &color = colors[i];
                ply_write(ply_file,
                        std::min(255.0, std::max(0.0, color(0) * 255.0)));
                ply_write(ply_file,
                        std::min(255.0, std::max(0.0, color(1) * 255.0)));
                ply_write(ply_file,
                        std::min(255.0, std::max(0.0, color(2) * 255.0)));
            }
            /*
            if(labels.size())
            {
                int semantic_label = get_semantic_label(labels[i]);
                int instance_label = get_instance_label(labels[i]);
                ply_write(ply_file, semantic_label);
                ply_write(ply_file, instance_label);
            }*/
        }
        for (size_t i = 0; i < triangles.size(); i++)
        {
            const auto &triangle = triangles[i];
            ply_write(ply_file, 3);
            ply_write(ply_file, triangle(0));
            ply_write(ply_file, triangle(1));
            ply_write(ply_file, triangle(2));
        }
        ply_close(ply_file);
        return true;
}
bool WritePLY(const std::string &filename, Point3fList &points, Point3fList &normals, 
    Point3fList &colors, std::vector<long> &labels)
{
    //write ply with instance and semantic label
    if (points.size() == 0) {

        return true;
    }
    //PLY_ASCII
    p_ply ply_file = ply_create(filename.c_str(), PLY_LITTLE_ENDIAN,
                                NULL, 0, NULL);
    if (!ply_file) {
        std::cout<<"cannot open file."<<std::endl;
        return false;
    }
    ply_add_comment(ply_file, "Created by Collaborative FlashFusion");
    ply_add_element(ply_file, "vertex",
                    static_cast<long>(points.size()));
    ply_add_property(ply_file, "x", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
    ply_add_property(ply_file, "y", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
    ply_add_property(ply_file, "z", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
    if(normals.size() > 0) {
        ply_add_property(ply_file, "nx", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "ny", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "nz", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
    }
    if (colors.size() > 0) {
        ply_add_property(ply_file, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
        ply_add_property(ply_file, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
        ply_add_property(ply_file, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
    }
    if(labels.size() > 0)
    {
        ply_add_property(ply_file, "semantic", PLY_INT32, PLY_INT32, PLY_INT32);
        ply_add_property(ply_file, "instance", PLY_INT32, PLY_INT32, PLY_INT32);
    }
    if (!ply_write_header(ply_file)) {
        std::cout<<"Unable to write header."<<std::endl;
        return false;
    }

    bool printed_color_warning = false;
    for (size_t i = 0; i < points.size(); i++) {
        const Eigen::Vector3f &point = points[i];
        ply_write(ply_file, point(0));
        ply_write(ply_file, point(1));
        ply_write(ply_file, point(2));
        if (normals.size()) {
            const Eigen::Vector3f &normal = normals[i];
            ply_write(ply_file, normal(0));
            ply_write(ply_file, normal(1));
            ply_write(ply_file, normal(2));
        }
        if (colors.size()) {
            const Eigen::Vector3f &color = colors[i];
            ply_write(ply_file,
                      std::min(255.0, std::max(0.0, color(0) * 255.0)));
            ply_write(ply_file,
                      std::min(255.0, std::max(0.0, color(1) * 255.0)));
            ply_write(ply_file,
                      std::min(255.0, std::max(0.0, color(2) * 255.0)));
        }
        if(labels.size())
        {
            int semantic_label = get_semantic_label(labels[i]);
            int instance_label = get_instance_label(labels[i]);
            ply_write(ply_file, semantic_label);
            ply_write(ply_file, instance_label);
        }
    }

    ply_close(ply_file);
    return true;
}
bool ReadPLYToChiselMesh(const std::string &filename, chisel::MeshPtr &mesh_ptr)
{
    return ReadPLY(filename, mesh_ptr->compact_vertices, mesh_ptr->compact_normals, mesh_ptr->compact_colors, mesh_ptr->triangles);
}
bool WritePLYFromChiselMesh(const std::string &filename, chisel::MeshPtr &mesh_ptr)
{
    return WritePLY(filename, mesh_ptr->compact_vertices, mesh_ptr->compact_normals, mesh_ptr->compact_colors, mesh_ptr->triangles);
}
bool ReadPLYToChiselPcd(const std::string &filename, chisel::PcdPtr &pcd_ptr)
{
    return ReadPLY(filename, pcd_ptr->vertices, pcd_ptr->normals, pcd_ptr->colors);
}
bool WritePLYFromChiselPcd(const std::string &filename, chisel::PcdPtr &pcd_ptr)
{
    return WritePLY(filename, pcd_ptr->vertices, pcd_ptr->normals, pcd_ptr->colors);
}
bool WritePLYFromChiselPcdWithLabel(const std::string &filename, chisel::PcdPtr &pcd)
{
    return WritePLY(filename, pcd->vertices, pcd->normals, pcd->colors, pcd->labels);
}