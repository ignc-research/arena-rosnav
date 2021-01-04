
#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_


#include <algorithm>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Path.h>





using std::vector;
class PlanningVisualization{

private:
    /* data */
    /* visib_pub is seperated from previous ones for different info */
    ros::NodeHandle node;
    ros::Publisher goal_pub_;
    ros::Publisher subgoal_pub_;
    ros::Publisher global_path_pub_;
    vector<ros::Publisher> pubs_;

    enum PLANNING_ID {
        GOAL = 1,
        SUBGOAL = 200,
        GLOBAL_PATH = 300,
    };  

public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle& nh);

    // draw basic shapes
    void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);

    void displayLineList(const vector<Eigen::Vector3d>& list1,double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);
    
    //draw
    void drawGoal(geometry_msgs::PoseStamped goal, double resolution, const Eigen::Vector4d& color, int id = 0);

    void drawSubgoal(geometry_msgs::PoseStamped subgoal, double resolution,const Eigen::Vector4d& color, int id=0);

    void drawGlobalPath(nav_msgs::Path global_path,double resolution,const Eigen::Vector4d& color, int id=0);

    Eigen::Vector4d getColor(double h, double alpha = 1.0);

    typedef std::shared_ptr<PlanningVisualization> Ptr;
};

#endif