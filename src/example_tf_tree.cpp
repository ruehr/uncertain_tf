#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace std;

//create a tf tree to play around with


int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_tf_tree");

    ros::NodeHandle node;

    tf::TransformBroadcaster tb;

    ros::Rate rate(30.0);


    double angle = 0;
    ros::Time start_time = ros::Time::now();


    while (node.ok())
    {

        tf::Transform trans;
        trans.setOrigin(tf::Vector3(0.7,0,0));
        trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), -M_PI / 5));
        tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/map","/base_link"));
        trans.setOrigin(tf::Vector3(0,0.2,0));
        trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), M_PI / 4));
        tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/base_link","/eye"));
        trans.setOrigin(tf::Vector3(0,0.5,0));
        trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), M_PI / 3));
        tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/eye","object"));
        trans.setOrigin(tf::Vector3(0.2,0,0));
        trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), M_PI / 2));
        tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/base_link","/gripper"));

        angle = (ros::Time::now() - start_time).toSec() * 10 / 180 * M_PI;

        trans.setOrigin(tf::Vector3(-1.5,0,0));
        trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), 0));
        tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/map","/table"));

        trans.setOrigin(tf::Vector3(.5,.5,.5));
        trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), angle));
        tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/table","/turntable"));

        trans.setOrigin(tf::Vector3(0.4,0.4,0.1));
        trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), 0));
        tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/turntable","/object_on_table"));


        rate.sleep();

        ros::spinOnce();

    }
    return 0;
};

