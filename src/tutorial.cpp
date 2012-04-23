#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <uncertain_tf/UncertainTransformListener.h>
#include <uncertain_tf/UncertainTransformBroadcaster.h>

#include <geometry_msgs/PoseArray.h>


using namespace uncertain_tf;
using namespace std;

void spam_tfs()
{
    tf::TransformBroadcaster tb;

    ros::Rate rt(5);
    while (ros::ok())
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

        rt.sleep();
    }

}

// add a frame 'turntable' that rotates in z and an object 'on' this table, rotating with it
void spam_turntable_tfs()
{
    tf::TransformBroadcaster tb;

    double angle = 0;
    ros::Time start_time = ros::Time::now();

    ros::Rate rt(25);
    while (ros::ok())
    {
        angle = (ros::Time::now() - start_time).toSec() * 10 / 180 * M_PI;

        tf::Transform trans;
        trans.setOrigin(tf::Vector3(-0.7,0,0));
        trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), angle));
        tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/map","/turntable"));

        trans.setOrigin(tf::Vector3(0.2,0.2,0.05));
        trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), 0));
        tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/turntable","/object_on_table"));

        rt.sleep();
    }

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf_listener");

    std::cout << "USAGE bin/execname frame_name var_x var_y var_z var_rot_z var_rot_y var_rot_x" << std::endl;

    if (argc < 2)
        exit(0);

    ros::NodeHandle node;

    tf::TransformListener listener;

    UncertainTransformBroadcaster utfb;

    StampedCovariance stc;
    stc.resize(6,6);
    stc.setZero();
    //set values
    for (int i = 0; i < argc - 2; ++i)
    {
        stc(i,i) = atof(argv[i+2]);
    }
    stc.frame_id_ = argv[1];
    stc.stamp_ = ros::Time::now();
    utfb.sendCovariance(stc);

    UncertainTransformListener ulistener;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Publisher poseArrPub = node.advertise<geometry_msgs::PoseArray>("sampled_poses", 100);
    ros::Publisher poseArrPubResampled = node.advertise<geometry_msgs::PoseArray>("resampled_poses", 100);
    ros::Publisher poseArrPubTimeGaussian = node.advertise<geometry_msgs::PoseArray>("sampled_time_gaussian", 100);

    boost::thread t1(spam_tfs);
    boost::thread t2(spam_turntable_tfs);

    ros::Duration(0.3).sleep();

    ros::Rate rate(30.0);

    ros::Time start_time = ros::Time::now();

    while (node.ok())
    {

        stc.stamp_ = ros::Time::now();

        StampedCovariance cov_to_send = stc;
        for (int i = 0; i < 6; ++i)
        {
            //cov_to_send(i,i) = fabs(sin((stc.stamp_ - start_time).toSec() * .5 )) * stc(i,i); // test time-varying covariance
            cov_to_send(i,i) = stc(i,i);
        }

        //std::cout << cov_to_send << std::endl;

        utfb.sendCovariance(cov_to_send);

        rate.sleep();

        ros::spinOnce();

        //sample
        if (1)
        {
            std::vector<StampedTransform> transform_samples;
            std::cout << endl << "***********************************************************************************" << endl << endl;

            ulistener.sampleTransform("map", "object_on_table", ros::Time(0), transform_samples, 50);

            geometry_msgs::PoseArray parr;
            parr.header.frame_id = transform_samples[0].frame_id_;
            parr.header.stamp = ros::Time(0);
            for (std::vector<StampedTransform>::iterator it = transform_samples.begin(); it != transform_samples.end(); ++it)
            {
                geometry_msgs::Pose ps;
                tf::poseTFToMsg(*it,ps);
                parr.poses.push_back(ps);
            }
            poseArrPub.publish(parr);
        }


        // resample from covariance estimated from sample set
        if (1)
        {
            std::vector<StampedTransform> transform_samples;

            ulistener.sampleTransform("map", "object_on_table", ros::Time(0), transform_samples, 50);

            MatrixXd sample_covar;
            MatrixXd samples = ulistener.sampleSetTFtoMatrixXd(transform_samples);
            //ulistener.calculateSampleCovariance(samples.transpose(), samples.transpose(), sample_covar);
            ulistener.calculateSampleCovariance(samples, samples, sample_covar);
            //std::cout << "sample covariance " << endl << sample_covar << endl;
            MatrixXd mean = ulistener.calculateSampleMean(samples);
            //std::cout << "sample mean" << endl << mean << endl;
            std::vector<tf::Transform> resampled;
            ulistener.sampleFromMeanCov(ulistener.transformVectorXdToTF(mean), sample_covar, resampled, 50);
            geometry_msgs::PoseArray parr_resampled;
            parr_resampled.header.frame_id = transform_samples[0].frame_id_;
            parr_resampled.header.stamp = ros::Time(0);
            for (std::vector<tf::Transform>::iterator it = resampled.begin(); it != resampled.end(); ++it)
            {
                geometry_msgs::Pose ps;
                tf::poseTFToMsg(*it,ps);
                parr_resampled.poses.push_back(ps);
            }
            poseArrPubResampled.publish(parr_resampled);

        }

        //sample from time a sec ago with .1 sec variance
        if (1)
        {
            std::vector<StampedTransform> transform_samples;
            ulistener.sampleTransformGaussianTime("map", "object_on_table", ros::Time::now() - ros::Duration(4),ros::Duration(2), transform_samples, 50);

            // note: the sampler catches all exeptions but does not generate a sample when catching one,
            // so the number of samples we get depends on how many fall inside a time period where the respective tfs are defined
            if (transform_samples.size() > 0)
            {

                geometry_msgs::PoseArray parr;
                parr.header.frame_id = transform_samples[0].frame_id_;
                parr.header.stamp = ros::Time(0);
                std::cout << "NUM SAMPLES TIME " << transform_samples.size() << std::endl;
                for (std::vector<StampedTransform>::iterator it = transform_samples.begin(); it != transform_samples.end(); ++it)
                {
                    geometry_msgs::Pose ps;
                    tf::poseTFToMsg(*it,ps);
                    parr.poses.push_back(ps);
                }

                poseArrPubTimeGaussian.publish(parr);
            }
        }


    }
    return 0;
};

