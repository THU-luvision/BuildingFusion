//
// Created by zhuyinheng on 19-6-18.
//

#include "SocketServer.h"





//float sample_pose[4 * 4] = {1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 12, 13, 14, 15, 1};
//cv::Mat sample_image = imread("/home/zhuyinheng/envs/opencv-3.3.0/samples/data/lena.jpg");


void SocketServer::start() {

    high_freq_thread_comm = boost::thread(boost::bind(&SocketServer::high_freq_socket, this));
    low_freq_thread_comm = boost::thread(boost::bind(&SocketServer::low_freq_socket, this));
//    low_freq_thread_comm.join();
}

void SocketServer::send_rbg_pose() {
//    convert frame
    std::vector<uchar> buffer;
    buffer.clear();
    cv::Mat frame;
    Eigen::Matrix4f cam_pose;

    mycache.get_rgb_pose(frame, cam_pose);


////    from flash fusion coordinate to unity coordinate: 1.transform into angleaxis 2. inverse y in angleaxis.axis 3. back to matrix4
//    Eigen::Matrix3f cam_pose_rotation;
//    cam_pose_rotation<<
//            cam_pose(0, 0),cam_pose(0, 1),cam_pose(0, 2),
//            cam_pose(1, 0),cam_pose(1, 1),cam_pose(1, 2),
//            cam_pose(2, 0),cam_pose(2, 1),cam_pose(2, 2);
//    Eigen::AngleAxisf cam_pose_angleaxis(cam_pose_rotation);
//    Eigen::Vector3f &cam_pose_axis = cam_pose_angleaxis.axis();
//    cam_pose_axis[1]*=-1.0;
//    Eigen::Matrix3f new_cam_pose_rotation;
//    new_cam_pose_rotation=cam_pose_angleaxis.toRotationMatrix().cast<float>();
//    Eigen::Matrix4f new_cam_pose;
//    new_cam_pose<<  new_cam_pose_rotation(0, 0),new_cam_pose_rotation(0, 1),new_cam_pose_rotation(0, 2),cam_pose(0,3),
//            new_cam_pose_rotation(1, 0),new_cam_pose_rotation(1, 1),new_cam_pose_rotation(1, 2),-1*cam_pose(1,3),
//            new_cam_pose_rotation(2, 0),new_cam_pose_rotation(2, 1),new_cam_pose_rotation(2, 2),cam_pose(2,3),
//            0,0,0,1;
    imencode(".jpeg", frame, buffer);

//    flatten pose
    float raw_pose[16];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
//            raw_pose[i * 4 + j] = new_cam_pose(i, j);
            raw_pose[i * 4 + j] = cam_pose(i, j);
        }
    }
//    convert pose
    uchar *pose_buffer = (uchar *) raw_pose;
    for (int i = 0; i < 64; i++) {
        buffer.push_back(pose_buffer[i]);
//        std::cout << pose_buffer[i] + '0' << " ";
    }
//    std::cout << "packed result on (4,4):" << *(float *) (buffer.data() + buffer.size() - 4) << std::endl;
//    std::cout << "packed result on (3,4):" << *(float *) (buffer.data() + buffer.size() - 4 * 5) << std::endl;

//    send
    int t = send(high_freq_connfd, buffer.data(), buffer.size(), 0);
//    std::cout << buffer.size() << std::endl;
}


void SocketServer::send_mesh() {





    map<long, CommMesh> mesh_array;
    mycache.split_mesh(mesh_array);

    if(mesh_array.empty())
    {
        if(send(this->low_freq_connfd, "error", 5, 0)<0)
        {
            std::cout<<"sending error error"<<std::endl;
            exit(-1);
        }
    }

// index definition:
// 1~21:length of each of 21 cate vertex list
// 22~42:length of each of 21 cate color list

    std::vector<int> index;
// buffer definition:
// for whole buffer [   cate 1      |    cate 2      |    cate ...      |       cate 20       ]
// for cate i | (vertex list) (color list) |
// for list (4 byte float: x,y,z)

    std::vector<float> send_buffer;
//    std::cout << "vertice: ";
    for (int i = 0; i < 20; ++i) {
        index.push_back(mesh_array[i].vertices.size());
//        std::cout << mesh_array[i].vertices.size() << "  ";
    }
//    std::cout << std::endl << "color :";
    for (int i = 0; i < 20; ++i) {
        index.push_back(mesh_array[i].color.size());
//        std::cout << mesh_array[i].color.size() << "  ";
    }
//    std::cout << std::endl;
    float *tmp_p = (float *) index.data();
    assert(index.size() == 40);
    for (int i = 0; i < index.size(); ++i) {
        send_buffer.push_back(tmp_p[i]);
    }

    /*NOTICE:
     * transform coordinate system here including flip y value
     * */
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < mesh_array[i].vertices.size(); ++j) {
            send_buffer.push_back(mesh_array[i].vertices[j][0]);
            send_buffer.push_back(-mesh_array[i].vertices[j][1]);
            send_buffer.push_back(mesh_array[i].vertices[j][2]);
        }
        for (int j = 0; j < mesh_array[i].color.size(); ++j) {
            send_buffer.push_back(mesh_array[i].color[j][0]);
            send_buffer.push_back(mesh_array[i].color[j][1]);
            send_buffer.push_back(mesh_array[i].color[j][2]);
        }
    }
    assert(sizeof(float)== sizeof(int));

//    send_buffer.clear();
//    send_buffer.resize(1024*1024*2);
//    std::cout << "size" << send_buffer.size() * sizeof(float) << std::endl;
//    std::cout << "buffer 1:" << ((int *) send_buffer.data())[0] << std::endl;


    assert(send_buffer.size() != 0);
    int send_buff_size=sizeof(float) * send_buffer.size();
    int patch_total = send_buff_size / patch_size;
    if (send_buff_size % patch_size!= 0) patch_total++;
//    std::cout<<"sending buffer size:"<<send_buff_size<<std::endl;
//    usleep(1000*30);
//    send(this->low_freq_connfd, &send_buff_size, sizeof(int), 0);
    int n=send(this->low_freq_connfd, send_buffer.data(), send_buff_size, 0);
    if(n==-1)
    {
        std::cout<<"errno "<<errno<<std::endl;
    }
    if (n!=send_buff_size)
    {
        std::cout<<"n:"<<n<<" send_buff_size:"<<send_buff_size<<std::endl;
    }
//    std::cout<<"send_succeed"<<send_buff_size<<std::endl;
//    int comfird_data_len=0;
//    while(send_buff_size>comfird_data_len)
//    {
//        char seg_sending_msg_buff[1024];
//        int n = recv(this->low_freq_connfd, seg_sending_msg_buff, 1024, 0);
//        std::string msg(seg_sending_msg_buff);
//        std::cout<<"recv："<<msg<<std::endl;
//        if(msg.find("getmeshmesh")!=msg.npos || msg.find("error")!=msg.npos)
//        {
//            std::cout<<"wrong comfirmed data "<<msg<<std::endl;
//            return;
//        }
//        comfird_data_len=atoi(msg.c_str());
//        int send_patch_length;
//        if(send_buff_size-comfird_data_len>=patch_size)
//        {
//            send_patch_length=patch_size;
//        } else{
//            send_patch_length=send_buff_size-comfird_data_len;
//        }
//        n=send(this->low_freq_connfd, (char *)&(((char *) send_buffer.data())[comfird_data_len]), send_patch_length, 0);
//        if(n==-1)
//        {
//            std::cout<<"errno "<<errno<<std::endl;
//        }
//        if (n!=send_patch_length)
//        {
//            std::cout<<"n:"<<n<<" patch length:"<<send_patch_length<<"comfired data:"<<comfird_data_len<<std::endl;
//        }
//        assert(n==send_patch_length);
//    }
}

void SocketServer::high_freq_socket() {


    char buff[MAXLINE];
    FILE *fp;
    int n;

    if ((high_freq_listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        return;
    }
    printf("----init high freq socket----\n");

    memset(&high_freq_servaddr, 0, sizeof(high_freq_servaddr));
    high_freq_servaddr.sin_family = AF_INET;
    high_freq_servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    high_freq_servaddr.sin_port = htons(high_freq_port);
    //设置端口可重用
    int contain = 1;
    setsockopt(high_freq_listenfd, SOL_SOCKET, SO_REUSEADDR, &contain, sizeof(int));

    if (bind(high_freq_listenfd, (struct sockaddr *) &high_freq_servaddr, sizeof(high_freq_servaddr)) == -1) {
        printf("bind socket error: %s(errno: %d)\n", strerror(errno), errno);
        return;
    }
    printf("----bind sucess----\n");

    if (listen(high_freq_listenfd, 1) == -1) {
        printf("listen socket error: %s(errno: %d)\n", strerror(errno), errno);
        return;
    }

    printf("======waiting for client's request======\n");
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t size = sizeof(client_addr);
        if ((high_freq_connfd = accept(high_freq_listenfd, (struct sockaddr *) &client_addr, &size)) == -1) {
            printf("accept socket error: %s(errno: %d)", strerror(errno), errno);
            continue;
        }
//        std::cout << "*********************connected**********************" << std::endl;
        double t = 0;
        while (1) {

//            std::cout << "*********************connected**********************" << std::endl;
//            std::cout << 1 / (((double) clock() - t) / CLOCKS_PER_SEC) << std::endl;
            t = clock();
            n = recv(high_freq_connfd, buff, MAXLINE, 0);
            std::string msg(buff);
//            std::cout << msg << "*******************************************************" << std::endl;
            if (msg.find("rgbpose") != msg.npos) {
//                std::cout << "to 1" << std::endl;
                send_rbg_pose();
            } else {
                send(high_freq_connfd, "error", 6, 0);
            }
        }
        close(high_freq_connfd);
        fclose(fp);

    }
    close(high_freq_listenfd);
    return;
}

void SocketServer::low_freq_socket() {

    char buff[MAXLINE];
    FILE *fp;
    int n;

    if ((low_freq_listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        return;
    }
    printf("----init high freq socket----\n");

    memset(&low_freq_servaddr, 0, sizeof(low_freq_servaddr));
    low_freq_servaddr.sin_family = AF_INET;
    low_freq_servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    low_freq_servaddr.sin_port = htons(low_freq_port);
    //设置端口可重用
    int contain;
    setsockopt(low_freq_listenfd, SOL_SOCKET, SO_REUSEADDR, &contain, sizeof(int));

    if (bind(low_freq_listenfd, (struct sockaddr *) &low_freq_servaddr, sizeof(low_freq_servaddr)) == -1) {
        printf("bind socket error: %s(errno: %d)\n", strerror(errno), errno);
        return;
    }
    printf("----bind sucess----\n");

    if (listen(low_freq_listenfd, 20) == -1) {
        printf("listen socket error: %s(errno: %d)\n", strerror(errno), errno);
        return;
    }

    int result=0;
    unsigned int len=4;

    int value=12082912;
    int tmpCode=0;
    tmpCode=::getsockopt(low_freq_listenfd,SOL_SOCKET, SO_RCVBUF, (char*)&result, &len);
    printf("SO_RCVBUF=%d, tmpCode=%d\n", result,tmpCode);
    tmpCode=::getsockopt(low_freq_listenfd,SOL_SOCKET, SO_SNDBUF, (char*)&result, &len);
    printf("SO_SNDBUF=%d, tmpCode=%d\n", result,tmpCode);
//
//    tmpCode=::setsockopt(low_freq_listenfd, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(value));
//    tmpCode=::setsockopt(low_freq_listenfd, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(value));
//
//    tmpCode=::getsockopt(low_freq_listenfd,SOL_SOCKET, SO_RCVBUF, (char*)&result, &len);
//    printf("SO_RCVBUF=%d, tmpCode=%d\n", result,tmpCode);
//    tmpCode=::getsockopt(low_freq_listenfd,SOL_SOCKET, SO_SNDBUF, (char*)&result, &len);
//    printf("SO_SNDBUF=%d, tmpCode=%d\n", result,tmpCode);
//
//


    printf("======waiting for client's request======\n");
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t size = sizeof(client_addr);
        if ((low_freq_connfd = accept(low_freq_listenfd, (struct sockaddr *) &client_addr, &size)) == -1) {
            printf("accept socket error: %s(errno: %d)", strerror(errno), errno);
            continue;
        }
//        std::cout << "*********************connected**********************" << std::endl;
        double t = 0;
        while (1) {

//            std::cout << "*********************connected**********************" << std::endl;
//            std::cout << 1 / (((double) clock() - t) / CLOCKS_PER_SEC) << std::endl;
            t = clock();
            n = recv(low_freq_connfd, buff, MAXLINE, 0);
            std::string msg(buff);
//            std::cout << msg << "*******************************************************" << std::endl;
            if (msg.find("getmeshmesh") != msg.npos) {
//                std::cout << "to 1" << std::endl;
                send_mesh();
            } else {
//                std::cout << "receve wrong getmeshmesh" << std::endl;
//                send(low_freq_connfd, "error", 6, 0);
            }
        }
        close(low_freq_connfd);
        fclose(fp);

    }
    close(low_freq_listenfd);
    return;
}

SocketServer::~SocketServer() {

    close(low_freq_connfd);
    close(low_freq_listenfd);
    close(high_freq_connfd);
    close(high_freq_listenfd);

}

void CommCache::get_rgb_pose(cv::Mat &frame, Eigen::Matrix4f &cam_pose) {
    boost::unique_lock<boost::mutex> lk(entry_mutex);
    frame = this->frame.clone();
    cam_pose = this->cam_pose;
//    std::cout << cam_pose << std::endl;
}

void CommCache::set_rgb_pose(cv::Mat &frame, Eigen::Matrix4f &cam_pose) {
    boost::unique_lock<boost::mutex> lk(entry_mutex);
    cv::Mat tmp= frame.clone();
    cv::cvtColor(tmp,this->frame,cv::COLOR_BGR2RGB);
//    cv::cvtColor(tmp,tmp,cv::COLOR_BGR2RGB);
//    cv::flip(tmp,this->frame,0);

    this->cam_pose = cam_pose;


}

void CommCache::get_mesh(CommMesh &mesh) {
    boost::unique_lock<boost::mutex> lk(mesh_mutex);
    mesh.set(this->mesh);

}

void CommCache::set_mesh(CommMesh &mesh) {

    boost::unique_lock<boost::mutex> lk(mesh_mutex);
    this->mesh.set(mesh);

}
