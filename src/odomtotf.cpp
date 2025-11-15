class odomtotf {
public:
    odomtotf() {
        ros::NodeHandle private_nh("~");
       
        odometry_sub_ = nh_.subscribe("/odometry", 1, &odomtotf::odomtfCallback, this);
        
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber odometry_sub;
    
    tf::TransformBroadcaster odom_broadcaster_;


void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {


        double x = msg-->point.x;
        double y = msg-->point.y;

        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "world";
        odom_tf.child_frame_id = "vehicle";

        odom_tf.transform.translation.x = x;
        odom_tf.transform.translation.y = y;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation = odom_quat;

        odom_broadcaster_.sendTransform(odom_tf);


}


int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_tf");
    odomtotf node;
    ROS_INFO("Nodo avviato correttamente!");

    ros::spin();
    return 0;
}