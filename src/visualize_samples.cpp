#include <ros/ros.h>
#include <tf/tf.h>

#include <uncertain_tf/UncertainTransformListener.h>

#include <geometry_msgs/PoseArray.h>


int main(int argc, char** argv)
{

    std::cout << "USAGE: visualize_samples target_frame source_frame num_samples pose_array_topic period(milliseconds)" << std::endl;

    std::cout << "argc" << argc << std::endl;

    if (argc < 6)
        exit(0);

    ros::init(argc, argv, "visualize_samples", ros::init_options::AnonymousName);

    ros::NodeHandle node;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher poseArrPub = node.advertise<geometry_msgs::PoseArray>(argv[4], 100);

    uncertain_tf::UncertainTransformListener ulistener;

    double period = atof(argv[5]);

    ros::Rate rate(1000 / period);

    int num_samples = atoi(argv[3]);

    std::cout << "num samples" << num_samples << std::endl;

    while (node.ok())
    {

        std::vector<StampedTransform> transform_samples;

        ulistener.sampleTransform(argv[1],argv[2], ros::Time(0), transform_samples, num_samples);

        geometry_msgs::PoseArray parr;

        parr.header.frame_id = argv[1];
        parr.header.stamp = ros::Time::now();
        for (std::vector<StampedTransform>::iterator it = transform_samples.begin(); it != transform_samples.end(); ++it)
        {
            geometry_msgs::Pose ps;
            tf::poseTFToMsg(*it,ps);
            parr.poses.push_back(ps);
        }
        poseArrPub.publish(parr);

        rate.sleep();
    }

}
