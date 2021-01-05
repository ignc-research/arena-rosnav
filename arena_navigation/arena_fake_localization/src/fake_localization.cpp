
#include <arena_fake_localization/fake_localization.h>


FakeLocalization::FakeLocalization(const ros::NodeHandle node){
      // init node handler
      m_nh=node;

      // init robot pose publisher 
      m_posePub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose",1,true);
      
      // init transform related
      m_tfServer = new tf2_ros::TransformBroadcaster();
      m_tfBuffer = new tf2_ros::Buffer();
      m_tfListener = new tf2_ros::TransformListener(*m_tfBuffer);

      // reset m_base_pos_received
      m_base_pos_received = false;


      // get ros param from /ns/node_name/..
      m_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
      m_nh.param("base_frame_id", base_frame_id_, std::string("base_link")); 
      m_nh.param("global_frame_id", global_frame_id_, std::string("map"));
      m_nh.param("delta_x", delta_x_, 0.0);
      m_nh.param("delta_y", delta_y_, 0.0);
      m_nh.param("delta_yaw", delta_yaw_, 0.0);      
      m_nh.param("transform_tolerance", transform_tolerance_, 0.1);
      m_nh.param("base_pose_ground_truth", base_pose_ground_truth_,std::string("/odometry/ground_truth"));
      
      // offset map to odom
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, -delta_yaw_);
      m_offsetTf = tf2::Transform(q, tf2::Vector3(-delta_x_, -delta_y_, 0.0));

      
      // create nodehnadle with /ns/..
      ros::NodeHandle nh;
      
      // create subscriber and filter
      stuff_sub_ = nh.subscribe(base_pose_ground_truth_, 100, &FakeLocalization::stuffFilter, this);
      filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "", 100); // this is a dummy message filter subscriber, the subscribed msg will be added manuelly
      filter_ = new tf2_ros::MessageFilter<nav_msgs::Odometry>(*filter_sub_, *m_tfBuffer, base_frame_id_, 100, nh);
      filter_->registerCallback(boost::bind(&FakeLocalization::update, this, _1));

      /*
      // subscription to "2D Pose Estimate" from RViz:
      m_initPoseSub = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh, "initialpose", 1);
      m_initPoseFilter = new tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*m_initPoseSub, *m_tfBuffer, global_frame_id_, 1, nh);
      m_initPoseFilter->registerCallback(boost::bind(&FakeLocalization::initPoseReceived, this, _1));
      */
    }

FakeLocalization::~FakeLocalization(void)
    {
      if (m_tfServer)
        delete m_tfServer; 
      if (m_tfListener)
        delete m_tfListener;
      if (m_tfBuffer)
        delete m_tfBuffer;
    }

void FakeLocalization::stuffFilter(const nav_msgs::OdometryConstPtr& odom_msg){
      //we have to do this to force the message filter to wait for transforms
      //from odom_frame_id_ to base_frame_id_ to be available at time odom_msg.header.stamp
      //really, the base_pose_ground_truth should come in with no frame_id b/c it doesn't make sense
      boost::shared_ptr<nav_msgs::Odometry> stuff_msg(new nav_msgs::Odometry);
      *stuff_msg = *odom_msg;
      stuff_msg->header.frame_id = odom_frame_id_;
      // manuelly add msg to filter
      filter_->add(stuff_msg);
    }

void FakeLocalization::update(const nav_msgs::OdometryConstPtr& message){
      tf2::Transform txi;
      tf2::convert(message->pose.pose, txi);
      txi = m_offsetTf * txi;

      geometry_msgs::TransformStamped odom_to_map;
      try
      {
        geometry_msgs::TransformStamped txi_inv;
        txi_inv.header.frame_id = base_frame_id_;
        txi_inv.header.stamp = message->header.stamp;
        tf2::convert(txi.inverse(), txi_inv.transform);

        m_tfBuffer->transform(txi_inv, odom_to_map, odom_frame_id_);
      }
      catch(tf2::TransformException &e)
      {
        ROS_ERROR("Failed to transform to %s from %s: %s\n", odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
        return;
      }

      geometry_msgs::TransformStamped trans;
      trans.header.stamp = message->header.stamp + ros::Duration(transform_tolerance_);
      trans.header.frame_id = global_frame_id_;
      trans.child_frame_id = message->header.frame_id;
      tf2::Transform odom_to_map_tf2;
      tf2::convert(odom_to_map.transform, odom_to_map_tf2);
      tf2::Transform odom_to_map_inv = odom_to_map_tf2.inverse();
      tf2::convert(odom_to_map_inv, trans.transform);
      m_tfServer->sendTransform(trans);

      

      tf2::Transform current;
      tf2::convert(message->pose.pose, current);

      //also apply the offset to the pose
      current = m_offsetTf * current;

      geometry_msgs::Transform current_msg;
      tf2::convert(current, current_msg);

      // Publish localized pose
      m_currentPos.header = message->header;
      m_currentPos.header.frame_id = global_frame_id_;
      tf2::convert(current_msg.rotation, m_currentPos.pose.pose.orientation);
      m_currentPos.pose.pose.position.x = current_msg.translation.x;
      m_currentPos.pose.pose.position.y = current_msg.translation.y;
      m_currentPos.pose.pose.position.z = current_msg.translation.z;
      m_posePub.publish(m_currentPos);



      // The particle cloud is the current position. Quite convenient.
      //m_particleCloud.header = m_currentPos.header;
      //m_particleCloud.poses[0] = m_currentPos.pose.pose;
      //m_particlecloudPub.publish(m_particleCloud);
    }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_localization");
  
  ros::NodeHandle private_nh("~");

  FakeLocalization robot_pose_node(private_nh);

  ros::spin();

  return 0;
}

