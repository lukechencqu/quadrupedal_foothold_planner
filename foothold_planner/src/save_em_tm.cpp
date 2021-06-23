#include <ros/ros.h>
#include <traversability_msgs/


// 全局坐标系名称
std::string frameID;
// 是否发布全局坐标系到base坐标系的TF变换的标志
bool publishTFFlag = false;


void gtCallback(const nav_msgs::OdometryConstPtr &msg){
    // recieved pose msg.

    // map2base tf.
    geometry_msgs::TransformStamped tf;
    static tf2_ros::TransformBroadcaster br;

    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = frameID;
    tf.child_frame_id = "base";
    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation.x = msg->pose.pose.orientation.x;
    tf.transform.rotation.y = msg->pose.pose.orientation.y;
    tf.transform.rotation.z = msg->pose.pose.orientation.z;
    tf.transform.rotation.w = msg->pose.pose.orientation.w;

    br.sendTransform(tf);

    // Odometry 2 PoseWithCovarrianceStampted 
    geometry_msgs::PoseWithCovarianceStamped gt;
    gt.header.stamp = ros::Time::now();
    gt.header.frame_id = frameID;
    gt.pose = msg->pose;

    // 发布机器人位姿，用于EM建图
    gtPub.publish(gt);
    // 发布世界坐标系（或odom)坐标系到机器人base坐标系的TF，用于加入世界坐标系并构建全局EM地图
    if(publishTFFlag)
        tfPub.publish(tf);
}


ros::ServiceClient tmClient;
tmClient.request.


int main(int argc, char** argv){
    ros::init(argc, argv, "map2base_tf");
    frameID = argv[1];
    publishTFFlag = argv[2];
    std::cout<<"publishTFFlag: "<<publishTFFlag<<std::endl;

    ros::NodeHandle node;

    ros::Subscriber gtSub = node.subscribe("/laikago_gazebo/ground_truth", 500, &gtCallback);

    tfPub = node.advertise<geometry_msgs::TransformStamped>("/map2base_tf", 500);
    gtPub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/laikago_gazebo/gt_pose", 500);

    ros::spin();

    return 0;
}