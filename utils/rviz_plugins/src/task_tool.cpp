#include "task_tool.h"


namespace rviz {

TaskTool::TaskTool() {}

// Disconnect the service client when the tool's destructor is called
TaskTool::~TaskTool() { 
    //task_service_.shutdown(); 
    }

// When the tool is initially loaded, connect to the pause toggle service
void TaskTool::onInitialize() {
  task_service_ = nh_.serviceClient<std_srvs::Empty>("task_generator");
  setName("Generate Task");
}

// Every time the user presses the tool's Rviz toolbar button, call the pause
// toggle service
void TaskTool::activate() {
  std_srvs::Empty empty;

  bool success=task_service_.call(empty);
  if(success){
      ROS_INFO("Task Generator:Success");
  }else{
      ROS_WARN("Task Generator:Failed");
  }
  
}

int TaskTool::processMouseEvent( ViewportMouseEvent& event ){
    return Finished;
}

void TaskTool::deactivate() {
    
}

}  // end namespace flatland_viz

// Tell pluginlib about the tool class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::TaskTool, rviz::Tool)