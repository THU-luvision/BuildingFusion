//
// Created by zhuyinheng on 19-6-18.
//

#ifndef MILD_SOCKETSERVER_H
#define MILD_SOCKETSERVER_H

#include<stdio.h>
#include<stdlib.h>
#include<cstring>
#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<unistd.h>
#include <map>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <opencv2/opencv.hpp>
#include "./../CHISEL/src/open_chisel/mesh/Mesh.h"
#include "./../GCSLAM/frame.h"




using namespace chisel;
class CommMesh{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:

    Vec3List vertices;
    Vec3List color;
    std::vector<long> label;


    CommMesh(){}

    void set(Vec3List vertices,
             Vec3List color,
    std::vector<long> label)
    {
        this->vertices=vertices;
        this->color=color;
        this->label=label;
    }

    void set(CommMesh &mesh)
    {
        this->vertices=mesh.vertices;
        this->color=mesh.color;
        this->label=mesh.label;
    }

};
class CommCache{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:

    CommCache()
    {
        cv::Mat sample_image = imread("/home/zhuyinheng/envs/opencv-3.3.0/samples/data/lena.jpg");
        this->frame=sample_image.clone();
        this->cam_pose=Eigen::MatrixXf::Ones(4,4);
    }
    void get_rgb_pose(cv::Mat &frame,Eigen::Matrix4f &cam_pose);
    void set_rgb_pose(cv::Mat &frame,Eigen::Matrix4f &cam_pose);

    void get_mesh(CommMesh& mesh);
    void set_mesh(CommMesh& mesh);
    void set_mesh(Vec3List &vertices,
                  Vec3List &color,
                  std::vector<long> &label)
    {
        boost::unique_lock<boost::mutex> lk(mesh_mutex);
//        if(vertices.size()>1000000) return;
        this->mesh.vertices.clear();
        this->mesh.color.clear();
        this->mesh.label.clear();
        this->mesh.vertices=vertices;
        this->mesh.color=color;
        this->mesh.label=label;
        std::cout<<"size"<<this->mesh.vertices.size()<<std::endl;
    }

    void split_mesh(map<long,CommMesh> &dst_mesh)
    {
        boost::unique_lock<boost::mutex> lk(mesh_mutex);
        int len=this->mesh.label.size();
        if( !( len>0 && len%3==0 && this->mesh.label.size() == this->mesh.vertices.size() && this->mesh.label.size()==this->mesh.color.size()))
        {
            return;
        }

//        split mesh according to label
        for (int i = 0; i <= 20; ++i) {
            CommMesh tmp;
            dst_mesh.insert(pair<long,CommMesh>(i,tmp));
        }
        for (int i = 0; i < len; i+=3) {
            if (i+2>=len)
                break;
            if (this->mesh.label[i]!=this->mesh.label[i+1] ||this->mesh.label[i]!=this->mesh.label[i+2] || this->mesh.label[i+1]!=this->mesh.label[i+2])
            {
                dst_mesh[this->mesh.label[i]].vertices.push_back(this->mesh.vertices[i]);
                dst_mesh[this->mesh.label[i]].vertices.push_back(this->mesh.vertices[i+1]);
                dst_mesh[this->mesh.label[i]].vertices.push_back(this->mesh.vertices[i+2]);
                dst_mesh[this->mesh.label[i]].color.push_back(this->mesh.color[i]);
                dst_mesh[this->mesh.label[i]].color.push_back(this->mesh.color[i+1]);
                dst_mesh[this->mesh.label[i]].color.push_back(this->mesh.color[i+2]);

                dst_mesh[this->mesh.label[i+1]].vertices.push_back(this->mesh.vertices[i]);
                dst_mesh[this->mesh.label[i+1]].vertices.push_back(this->mesh.vertices[i+1]);
                dst_mesh[this->mesh.label[i+1]].vertices.push_back(this->mesh.vertices[i+2]);
                dst_mesh[this->mesh.label[i+1]].color.push_back(this->mesh.color[i]);
                dst_mesh[this->mesh.label[i+1]].color.push_back(this->mesh.color[i+1]);
                dst_mesh[this->mesh.label[i+1]].color.push_back(this->mesh.color[i+2]);

                dst_mesh[this->mesh.label[i+2]].vertices.push_back(this->mesh.vertices[i]);
                dst_mesh[this->mesh.label[i+2]].vertices.push_back(this->mesh.vertices[i+1]);
                dst_mesh[this->mesh.label[i+2]].vertices.push_back(this->mesh.vertices[i+2]);
                dst_mesh[this->mesh.label[i+2]].color.push_back(this->mesh.color[i]);
                dst_mesh[this->mesh.label[i+2]].color.push_back(this->mesh.color[i+1]);
                dst_mesh[this->mesh.label[i+2]].color.push_back(this->mesh.color[i+2]);
            }
            else
            {
                dst_mesh[this->mesh.label[i]].vertices.push_back(this->mesh.vertices[i]);
                dst_mesh[this->mesh.label[i]].vertices.push_back(this->mesh.vertices[i+1]);
                dst_mesh[this->mesh.label[i]].vertices.push_back(this->mesh.vertices[i+2]);
                dst_mesh[this->mesh.label[i]].color.push_back(this->mesh.color[i]);
                dst_mesh[this->mesh.label[i]].color.push_back(this->mesh.color[i+1]);
                dst_mesh[this->mesh.label[i]].color.push_back(this->mesh.color[i+2]);
            }
        }
    }


private:
    mutable boost::mutex entry_mutex;
    mutable boost::mutex mesh_mutex;
    cv::Mat frame;
    Eigen::Matrix4f cam_pose;
    CommMesh mesh;
};
class SocketServer {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    ~SocketServer();
    void start();
    void high_freq_socket();
    void low_freq_socket();
    void send_rbg_pose();
    void send_mesh();
    CommCache mycache;

private:
    const int patch_size=1024*1024*1;
    const int high_freq_port=12345;
    const int low_freq_port=12346;
    bool connected;
    struct sockaddr_in  high_freq_servaddr;
    struct sockaddr_in  low_freq_servaddr;
    int  high_freq_listenfd, high_freq_connfd;
    int  low_freq_listenfd, low_freq_connfd;
    const int MAXLINE = 600;
    boost::thread high_freq_thread_comm;
    boost::thread low_freq_thread_comm;
};

#endif //MILD_SOCKETSERVER_H
