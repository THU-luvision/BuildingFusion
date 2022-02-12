#include <ros/ros.h>
#include <collaborative_fusion/Termination.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "terminate_srv"); 
    ros::NodeHandle n;
    ros::ServiceClient client_t = n.serviceClient<collaborative_fusion::Termination>("server_termination");
    collaborative_fusion::Termination srv;
    srv.request.terminate = true;
    if(client_t.call(srv))
    {
        std::cout <<"[INFO]::Terminate server successfully."<<std::endl;
        return 0;
    }
    else
    {
        std::cout <<"[ERROR]::Fail to terminate."<<std::endl;
        return 1;
    }    

}