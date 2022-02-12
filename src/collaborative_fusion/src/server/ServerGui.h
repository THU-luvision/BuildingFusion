#ifndef SERVER_GUI_H
#define SERVER_GUI_H

#include "../Shaders/Shaders.h"

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <GL/freeglut.h>
#include <GL/glut.h>
#include <GL/glew.h>

#include "../GCFusion/MobileGUI.hpp"
#include "../GCSLAM/MultiViewGeometry.h"
#include "ServerSLAM.h"
#define GLOBAL_MODLE_VERTEX_NUM (1024*1024*20)
namespace Server 
{
//
static float camera_color_table[] = 
{
    1,0,0,
    0,0,1,
    0,1,0
};

class ServerGui
    {
    public:
    MobileGUI* gui;            
    unsigned char * tsdf_visualization_buffer;    
    int  amount_tsdf_vertice_num =0;
    int byte_per_point=30;
    
    std::vector< std::vector<unsigned char> > meshes;
    MultiViewGeometry::CameraPara camera;
    std::vector<pangolin::GlTexture> imageTextures;
    std::shared_ptr<Shader> drawVoxelHashingStyle;
    std::shared_ptr<Shader> drawProgram;
    std::shared_ptr<Shader> drawPhongLighting;
    GLuint vbo;
    GLuint vbo_point_cloud;
    GLuint vbo_data_transfer;
    GLuint feedback_vbo;
    GLuint unstable_vbo;
    bool global_vertex_data_updated = 0;
    bool has_data = false;
    ServerGui()
    {

        tsdf_visualization_buffer = new unsigned char [GLOBAL_MODLE_VERTEX_NUM*byte_per_point];

    }
    void setCameraNum(int camera_num)
    {
        gui = new MobileGUI(0);
        imageTextures.resize(MultiViewGeometry::g_para.camera_num);

        int argc = 1;
        char ProjectName[256] = "MobileFusionServer";
        char *argv = ProjectName;
        glutInit(&argc, &argv);
        glutInitDisplayMode(GLUT_SINGLE);
        GLenum err=glewInit();
        if(err!=GLEW_OK) {
          exit(1);
        }

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER,vbo);
        glBufferData(GL_ARRAY_BUFFER, GLOBAL_MODLE_VERTEX_NUM * byte_per_point, &tsdf_visualization_buffer[0], GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glGenBuffers(1, &vbo_data_transfer);
        glGenBuffers(1, &unstable_vbo);
        glGenTransformFeedbacks(1, &feedback_vbo);
        glGenBuffers(1, &vbo_point_cloud);


        for(int cID = 0;cID!= MultiViewGeometry::g_para.camera_num;++cID)
        {
            imageTextures[cID] =pangolin::GlTexture(640,480,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
        }
        meshes.resize(MultiViewGeometry::g_para.camera_num,std::vector<unsigned char>());
        drawVoxelHashingStyle = loadProgramFromFile("draw_feedback_VoxelHashing.vert","draw_feedback_VoxelHashing.frag");
    }

    void show(ServerSLAM &server_slam)
    {
        if(!has_data)
        return;
        
        if(gui->followPose->Get())
        {
            Eigen::Matrix4d mv;
            mv <<   0.637559,  0.00426117,   -0.770392,      1.0641,
            0.770251,   0.0162343,    0.637531,     1.00111,
            0.0152234,   -0.999859,  0.00706816,    -16.3062,
            -2.249e-07, -5.0405e-06,   3.113e-08,     1.00021;
            setModelViewMatrix(mv);
        // Eigen::Matrix4f currPose = Eigen::Matrix4f::Identity();
        // gui->setModelView(currPose, camera.c_fy < 0);
        }

        //std::cout<<"global_vertex_data_updated: "<<global_vertex_data_updated<<std::endl;
        gui->PreCall();
        //std::cout<<"global_vertex_data_updated: "<<global_vertex_data_updated<<std::endl;
        if(gui->drawGlobalModel->Get())
        {
            //int draw_camera = 0;
            for(int camera_id = server_slam.rooms_for_each_camera.size() - 1; camera_id >=0 ; --camera_id)
            {
                if(server_slam.is_terminated[camera_id] == false && server_slam.keyframes[camera_id].size() > 0)
                {
                    int color_index = camera_id % 3;
                    gui->drawCamera(server_slam.cameraStartPoses[camera_id] * 
                        server_slam.submapPosesFinal[camera_id].back() * 
                        server_slam.keyframes[camera_id].rbegin()->second.pose_sophus[2],
                        camera_color_table[color_index * 3 ],  
                        camera_color_table[color_index * 3 + 1], 
                        camera_color_table[color_index * 3 + 2]);
                }
            }
            MobileShow();
        }

        

        gui->PostCall();
    }
    void updateMeshes(int cameraID,std::vector<unsigned char> &buffer)
    {
        meshes[cameraID] = buffer;
    }
    //@discarded
    void updateMeshes(int cameraID, ServerPointCloud &s_pcd, PoseSE3d &cameraStartPose)
    {
        s_pcd.transform_to_buffer(meshes[cameraID],cameraStartPose.matrix().cast<float>());
    }

    void updateMeshes(int cameraID, std::vector<ServerPointCloud> &s_pcds, PoseSE3dList &submapPosesRelativeChanges, PoseSE3d &cameraStartPose)
    {
        meshes[cameraID].clear();
        if(submapPosesRelativeChanges.size() != s_pcds.size())
        {
            std::cout<<"Submap size is not the same, there may be some delay."<<std::endl;
        }
        for(int i = 0; i< s_pcds.size() && i < submapPosesRelativeChanges.size(); ++i)
            s_pcds[i].transform_to_buffer(meshes[cameraID], 
                (cameraStartPose * submapPosesRelativeChanges[i]).matrix().cast<float>());
    }
    size_t putMeshInBuffer()
    {
        unsigned char *start = tsdf_visualization_buffer;
        size_t all = 0;
        /*
        for(int i = 0; i <meshes[0].size();++i)
        std::cout<<meshes[0][i];
        std::cout<<std::endl;
        */
        for(int i = 0;i!=MultiViewGeometry::g_para.camera_num;++i)
        {
            if(i > 0)
            start = start + meshes[i-1].size();
            //std::cout<<"meshes: "<<meshes[i].size()<<std::endl;
            memcpy(start, &meshes[i][0], meshes[i].size() );    

            all += meshes[i].size()/byte_per_point;

        //std::cout<<"all: "<<all<<std::endl;
        }
        amount_tsdf_vertice_num = all;
        std::cout<<"amount_tsdf_vertice_num: "<<amount_tsdf_vertice_num<<std::endl;

        return  all;
    }
    void setModelViewMatrix(const Eigen::Matrix4d &mat)
    {
        pangolin::OpenGlMatrix mv;
        memcpy(&mv.m[0], mat.data(), sizeof(Eigen::Matrix4d));
        gui->s_cam.SetModelViewMatrix(mv);
    }

    inline int MobileShow()
    {
        std::cout<<"global_vertex_data_updated: "<<global_vertex_data_updated<<std::endl;
        pangolin::OpenGlMatrix mvp = gui->s_cam.GetProjectionModelViewMatrix();
        
        //Eigen::Matrix4d mvp_eigen(mvp.m);
        //std::cout<<mvp_eigen<<std::endl;
        const float threshold = 3 ;
        //const bool drawUnstable=gui->drawUnstable->Get();
        const bool drawNormals=gui->drawNormals->Get();
        const bool drawColors=gui->drawColors->Get();
        const bool drawSemantic=gui->drawSemanticLabel->Get();
        const bool drawInstance=gui->drawInstanceLabel->Get();
        std::shared_ptr<Shader> program = drawVoxelHashingStyle;
        program->Bind();                    // set this program as current program
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        program->setUniform(Uniform("MVP", mvp));
        program->setUniform(Uniform("pose", pose));
        program->setUniform(Uniform("threshold", threshold));
        int colortype = 0;
        if(drawInstance)
        colortype = 4;
        else if(drawSemantic)
        colortype = 3;
        else if(drawNormals)
        colortype = 2;
        else if(drawColors)
        colortype = 1;
        else colortype = 0;//draw phong shading

        program->setUniform(Uniform("colorType", colortype));


        if(program == drawPhongLighting)
        {
            program->setUniform(Uniform("view_matrix", pose));
            program->setUniform(Uniform("proj_matrix", mvp));
            Eigen::Vector3f Lightla(0.2f, 0.2f, 0.2f);
            Eigen::Vector3f Lightld(1.0f, 1.0f, 1.0f);
            Eigen::Vector3f Lightls(1.0f, 1.0f, 1.0f);
            Eigen::Vector3f Lightldir(0.0, 0.0, 1.0f);
            Eigen::Vector3f fma(0.26f, 0.26f, 0.26f);
            Eigen::Vector3f fmd(0.35f, 0.35f, 0.35f);
            Eigen::Vector3f fms(0.30f, 0.30f, 0.30f);
            float fss = 16.0f;
            Eigen::Vector3f bma(0.85f, 0.85f, 0.85f);
            Eigen::Vector3f bmd(0.85f, 0.85f, 0.85f);
            Eigen::Vector3f bms(0.60f, 0.60f, 0.60f);
            float bss = 16.0f;

            Eigen::Matrix4f user_view_matrix = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f user_light_matrix = Eigen::Matrix4f::Identity();
            Eigen::Vector4f user_rot_center = Eigen::Vector4f(0, 0, 0, 1);
            program->setUniform(Uniform("Lightla", Lightla));
            program->setUniform(Uniform("Lightld", Lightld));
            program->setUniform(Uniform("Lightls", Lightls));
            program->setUniform(Uniform("Lightldir", Lightldir));
            program->setUniform(Uniform("fma", fma));
            program->setUniform(Uniform("fmd", fmd));
            program->setUniform(Uniform("fms", fms));
            program->setUniform(Uniform("bma", bma));
            program->setUniform(Uniform("bmd", bmd));
            program->setUniform(Uniform("bms", bms));
            program->setUniform(Uniform("bss", bss));
            program->setUniform(Uniform("fss", fss));
            program->setUniform(Uniform("user_view_matrix", user_view_matrix));
            program->setUniform(Uniform("user_light_matrix", user_light_matrix));
            program->setUniform(Uniform("user_rot_center", user_rot_center));
        }

        if(program == drawVoxelHashingStyle)
        {
            float s_materialShininess = 16.0f;
            Eigen::Vector4f s_materialAmbient   = Eigen::Vector4f(0.75f, 0.65f, 0.5f, 1.0f);
            Eigen::Vector4f s_materialDiffuse   = Eigen::Vector4f(1.0f, 0.9f, 0.7f, 1.0f);
            Eigen::Vector4f s_materialSpecular  = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
            Eigen::Vector4f s_lightAmbient 	    = Eigen::Vector4f(0.4f, 0.4f, 0.4f, 1.0f);
            Eigen::Vector4f s_lightDiffuse 		= Eigen::Vector4f(0.6f, 0.52944f, 0.4566f, 0.6f);
            Eigen::Vector4f s_lightSpecular 	= Eigen::Vector4f(0.3f, 0.3f, 0.3f, 1.0f);
            Eigen::Vector3f lightDir 	= Eigen::Vector3f(0.0f, -1.0f, 2.0f);

            program->setUniform(Uniform("materialShininess", s_materialShininess));
            program->setUniform(Uniform("materialAmbient", s_materialAmbient));
            program->setUniform(Uniform("materialDiffuse", s_materialDiffuse));
            program->setUniform(Uniform("materialSpecular", s_materialSpecular));
            program->setUniform(Uniform("lightAmbient", s_lightAmbient));
            program->setUniform(Uniform("lightDiffuse", s_lightDiffuse));
            program->setUniform(Uniform("lightSpecular", s_lightSpecular));
            program->setUniform(Uniform("lightDir", lightDir));
        }
        //This is for the point shader
        //setup a uniform:

    //    GLuint loc = glGetUniformLocation(program->programId(), "camera_array");
    //    glUniformMatrix4fv(loc, 26, false, camera_array_matrices, 0);


        std::cout<<"global_vertex_data_updated: "<<global_vertex_data_updated<<std::endl;
        if(global_vertex_data_updated)
        {

            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBufferSubData(GL_ARRAY_BUFFER,0,(amount_tsdf_vertice_num) * byte_per_point,tsdf_visualization_buffer);

            glBindBuffer(GL_ARRAY_BUFFER, 0);         
            global_vertex_data_updated = 0;
            // begin draw signed distance fields
        }


        glBindBuffer(GL_ARRAY_BUFFER, vbo);

        //glEnableVertexAttribArray(0);
        /*glVertexAttribPointer(index,size,type,normalized,stride,pointer)
        
        index: index
        size: every vpo's size
        type: the data type
        normalize: if data is normailized,true
        stride: offset 
        pointer: the first vertex offset
        
        */ 

        //glVertexAttribPointer(0, 3, GL_HALF_FLOAT, GL_FALSE, byte_per_point, 0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, byte_per_point, 0);//vertex
        glEnableVertexAttribArray(0);

        //glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 12 * sizeof(float), reinterpret_cast<GLvoid*>(sizeof(double) * 3));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,byte_per_point, reinterpret_cast<GLvoid*>(12));
        glEnableVertexAttribArray(1);
        

        glVertexAttribPointer(2, 3, GL_HALF_FLOAT, GL_TRUE, byte_per_point,reinterpret_cast< GLvoid*>(24));
        glEnableVertexAttribArray(2);


        glDrawArrays(GL_POINTS,0,amount_tsdf_vertice_num );

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);

        glBindBuffer(GL_ARRAY_BUFFER, 0);

        program->Unbind();


        float vertex_list[24] =
        {
            -0.5f, -0.5f, -0.5f,
            0.5f, -0.5f, -0.5f,
            -0.5f, 0.5f, -0.5f,
            0.5f, 0.5f, -0.5f,
            -0.5f, -0.5f, 0.5f,
            0.5f, -0.5f, 0.5f,
            -0.5f, 0.5f, 0.5f,
            0.5f, 0.5f, 0.5f,
        };
        GLint index_list[24] =
        {
            0, 1,
            2, 3,
            4, 5,
            6, 7,
            0, 2,
            1, 3,
            4, 6,
            5, 7,
            0, 4,
            1, 5,
            7, 3,
            2, 6
        };

        return 0;
    }
    };
    static ServerGui server_gui;
};

#endif