//
// Created by zhuyinheng on 19-6-19.
//

#include "SocketServer.h"

using namespace chisel;
void thread_runing(SocketServer &server)
{
// require access to mesh, pose, rgb frame
    while(1)
    {
        std::string msg=server.wait();
        if (msg=="askply")
        {
//            server.sendsocket()
        }
    }
}
int main()
{

    std::vector<Vec3f> sample;



    Mesh sample_ply;
//    sample_ply.load("");
    SocketServer server;
    server.ini(12345);


    return 0;
}