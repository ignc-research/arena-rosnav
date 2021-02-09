#include <arena_plan_manager/plan_manager.h>




int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_manager");
    std::string ns = ros::this_node::getNamespace();
    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    ros::NodeHandle node_handle;
    //ros::WallRate r(100);
    PlanManager plan_manager;
    plan_manager.init(node_handle);
    ROS_INFO_STREAM( ns<<":\tplan manager loaded!!");
    ros::spin();
}