#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <uncertain_tf/UncertainTransformListener.h>
#include <uncertain_tf/UncertainTransformBroadcaster.h>

#include <geometry_msgs/PoseArray.h>

using namespace uncertain_tf;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;

    tf::TransformListener listener;

    UncertainTransformBroadcaster utfb;

    UncertainTransformListener ulistener;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    //! time 1: object is observed in base_link
    {
        tf::Transform trans;
        trans.setOrigin(tf::Vector3(0,0,0));
        trans.setRotation(tf::Quaternion(0,0,0,1));
        utfb.sendTransform(tf::StampedTransform(trans,ros::Time(1),"/map","/base_link"));
    }

    {
        tf::Transform trans;
        trans.setOrigin(tf::Vector3(1,0,0));
        trans.setRotation(tf::Quaternion(0,0,0,1));
        utfb.sendTransform(tf::StampedTransform(trans,ros::Time(1),"/base_link","/object"));
    }

    {
        tf::Transform trans;
        trans.setOrigin(tf::Vector3(1,0,0));
        trans.setRotation(tf::Quaternion(0,0,0,1));
        utfb.sendTransform(tf::StampedTransform(trans,ros::Time(1),"/base_link","/gripper"));
    }

    {
        //add some uncertainty to map->base_link
        StampedCovariance stc;
        stc.resize(6,6);
        stc.setZero();

        // variance in x,x and y,y
        stc(0,0) = 0.0025;
        stc(1,1) = 0.0025;

        stc.frame_id_ = "/base_link";
        stc.stamp_ = ros::Time(1);
        utfb.sendCovariance(stc);
    }

    ros::Duration(0.1).sleep();

    {
        std::vector<StampedTransform> transform_samples;
        ulistener.sampleTransform("/gripper", "/object", ros::Time(1), transform_samples, 10);
        for (std::vector<StampedTransform>::iterator it = transform_samples.begin(); it != transform_samples.end(); ++it)
        {
            std::cout << (*it).getOrigin().x() << " " << (*it).getOrigin().y() << " "<< (*it).getOrigin().z() << std::endl;
        }
    }


    //! time 2: robot moved, query object in base_link again
    {
        tf::Transform trans;
        trans.setOrigin(tf::Vector3(0,0,0));
        trans.setRotation(tf::Quaternion(0,0,0,1));
        utfb.sendTransform(tf::StampedTransform(trans,ros::Time(2),"/map","/base_link"));
    }

    {
        tf::Transform trans;
        trans.setOrigin(tf::Vector3(1,0,0));
        trans.setRotation(tf::Quaternion(0,0,0,1));
        utfb.sendTransform(tf::StampedTransform(trans,ros::Time(2),"/base_link","/gripper"));
    }

    {
        //add some uncertainty to map->base_link
        StampedCovariance stc;
        stc.resize(6,6);
        stc.setZero();

        // variance in x,x and y,y
        stc(0,0) = 0.0025;
        stc(1,1) = 0.0025;

        stc.frame_id_ = "/base_link";
        stc.stamp_ = ros::Time(2);
        utfb.sendCovariance(stc);
    }

    ros::Duration(0.1).sleep();

    std::cout << " now checking via time travel: " << std::endl;

    {
        std::vector<StampedTransform> transform_samples;
        //ulistener.sampleTransform("/gripper", "/object", ros::Time(2), transform_samples, 10);
        ulistener.sampleTransform("/gripper", ros::Time(2), "/object", ros::Time(1), "/map", transform_samples, 10);
        for (std::vector<StampedTransform>::iterator it = transform_samples.begin(); it != transform_samples.end(); ++it)
        {
            std::cout << (*it).getOrigin().x() << " " << (*it).getOrigin().y() << " "<< (*it).getOrigin().z() << std::endl;
        }
    }

    return 0;
};

