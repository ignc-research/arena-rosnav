#include <flatland_plan_manager/plan_manager.h>




int main(int argc, char** argv) {
    std::cout<<"Plan manager node start"<<std::endl;
    ros::init(argc, argv, "plan_manager");
    
    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    ros::NodeHandle node_handle("~");
    //ros::WallRate r(100);
    ROS_INFO("abcd");
    PlanManager plan_manager;
    plan_manager.init(node_handle);
    ROS_INFO("abcd111");
    
    ros::spin();
}