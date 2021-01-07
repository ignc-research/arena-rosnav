uint32 ped_id
ford_msgs/Pose2DStamped[] traj
float32 value #Used for PREDICTION types
uint8 type 
#ENUM for type
uint8 OBSERVATION=0
uint8 PREDICTION_LINEAR=1
uint8 PREDICTION_GP=2