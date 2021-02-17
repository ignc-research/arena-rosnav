#include <arena_plan_manager/robot_state.h>
#include <arena_plan_manager/plan_collector.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <ros/ros.h>


bool generate_global_plan(ros::ServiceClient global_plan_client){
    //nav_msgs::GetPlan srv;
    std::cout<<"COME to plan global0"<<std::endl;
    arena_plan_msgs::MakeGlobalPlan srv;

    int iy,ix;
    srand (time(NULL));
    iy = rand() % 10 + 1;
    ix = rand() % 10 + 1;

    srv.request.tolerance=0.3;
    srv.request.start.header.frame_id="";
    srv.request.goal.header.frame_id="";
    srv.request.start.header.stamp={};
    srv.request.goal.header.stamp={};
    srv.request.goal.pose.orientation.w=0.0;
    srv.request.start.pose.orientation.w=0.0;
    srv.request.goal.pose.orientation.z=0.0;
    srv.request.start.pose.orientation.z=0.0;
    srv.request.goal.pose.position.x=double(ix);
    srv.request.start.pose.position.x=0.0;
    srv.request.goal.pose.position.y=double(iy);
    srv.request.start.pose.position.y=0.0;
    
    std::cout<<"COME to plan global1"<<std::endl;
    std::cout<<srv.request.start<<std::endl;
    std::cout<<srv.request.goal<<std::endl;
    
    if (global_plan_client.call(srv))  
    {   std::cout<<"COME to plan global2"<<std::endl;
        // set global_plan
        std::cout<<srv.response.plan.poses[0]<<std::endl;
        std::cout<<"COME to plan global3"<<std::endl;
        
        return true;
    }
    else{
        return false;
    }
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "abc");
    
    
    ros::NodeHandle nh("");
    //ros::WallRate r(100);
    std::string global_plan_service_name = "/sim_01/global_kino_make_plan";///move_base/NavfnROS/make_plan"; 
    ros::service::waitForService(global_plan_service_name); 
    ros::ServiceClient global_plan_client_= nh.serviceClient<arena_plan_msgs::MakeGlobalPlan>(global_plan_service_name);
    
    ros::Rate r(10);
    ros::Time ros_time;
    while (ros::ok())
    {   std::cout<<"start--------------"<<std::endl;
        RobotStatePtr s_start, s_end;
        std::cout<<generate_global_plan(global_plan_client_)<<std::endl;
        ros_time=ros::Time::now();
        std::cout<<ros_time<<std::endl;
        std::cout<<"end--------------"<<std::endl;
        ros::spinOnce();                   // Handle ROS events
        //std::cout<<"end2"<<std::endl;
        r.sleep();
    }

    
    

    
}