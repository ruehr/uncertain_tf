#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/time_cache.h>
#include <tf/tf.h>
#include <Eigen/Eigenvalues>
#include <uncertain_tf/CovarianceStamped.h>
#include <uncertain_tf/utfMessage.h>

#include <uncertain_tf/UncertainTransformListener.h>
#include <uncertain_tf/UncertainTransformBroadcaster.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>

//#include <EigenMultiVariateNormal.hpp>


using namespace uncertain_tf;
using namespace std;

void spam_tfs()
{
    tf::TransformBroadcaster tb;

    ros::Rate rt(5);
    while (ros::ok())
    {
        //(const tf::Transform& input, const ros::Time& timestamp, const std::string & frame_id, const std::string & child_frame_id):
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

        //trans.setOrigin(tf::Vector3(0,0,0));
        //trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), 0));
        //tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/object","object1"));

        //trans.setOrigin(tf::Vector3(0.2,0,0));
        //trans.setRotation(tf::Quaternion(tf::Vector3(0,0,1), M_PI / 2));
        //tb.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/gripper","/finger"));
        rt.sleep();
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;

    tf::TransformListener listener;

    UncertainTransformBroadcaster utfb;
    StampedCovariance stc;
    stc.resize(6,6);
    stc.setZero();
    //stc(2,2) = .1;
    for (int i = 0; i < argc - 2; ++i)
    {
        stc(i,i) = atof(argv[i+2]);
    }
    stc.frame_id_ = argv[1];
    stc.stamp_ = ros::Time::now();
    utfb.sendCovariance(stc);

    //ros::Subscriber utfsubsc = node.subscribe<uncertain_tf::utfMessage>("/tf", 100, boost::bind(&TransformListener::subscription_callback, this, _1)); ///\todo magic number
    //ros::Subscriber utfsubsc = node.subscribe<uncertain_tf::utfMessage>("/tf_uncertainty", 100, subscription_callback);
    UncertainTransformListener ulistener;

    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();

    ros::Publisher poseArrPub = node.advertise<geometry_msgs::PoseArray>("sampled_poses", 100);
    ros::Publisher poseArrPubResampled = node.advertise<geometry_msgs::PoseArray>("resampled_poses", 100);

    boost::thread t1(spam_tfs);

    ros::Duration(0.3).sleep();

    ros::Rate rate(3.0);
    while (node.ok())
    {

        stc.stamp_ = ros::Time::now();
        utfb.sendCovariance(stc);

        /*
        uncertain_tf::utfMessage utfm;
        uncertain_tf::CovarianceStamped cov_stamped_msg;
        uncertain_tf::covarianceStampedTFToMsg(stc, cov_stamped_msg);
        utfm.covariances.push_back(cov_stamped_msg);

        ulistener.subscription_callback(boost::make_shared<uncertain_tf::utfMessage>(utfm));
        */

        rate.sleep();

        ros::spinOnce();
        /*

        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform(argv[1], argv[2], ros::Time(0), transform);
            geometry_msgs::TransformStamped msg;
            tf::transformStampedTFToMsg(transform,msg);
            //std::cout << msg << std::endl;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }*/

        //cout << listener.allFramesAsDot() << endl;
        /*std::vector<std::string> vec;
        listener.getFrameStrings(vec);
        for (std::vector<std::string>::iterator it = vec.begin(); it != vec.end(); ++it)
        {
            std::string parent;
            listener.getParent(*it, ros::Time(0), parent);
            cout << "f: " << *it << " parent \'" << parent << "\'" << endl;
        }*/

        std::vector<StampedTransform> transform_samples;
        //ulistener.sampleTransform("gripper", "object", ros::Time(0), transform_samples, 10);
        std::cout << endl << "***********************************************************************************" << endl << endl;
        ulistener.sampleTransform("map", "object", ros::Time(0), transform_samples, 100);

        geometry_msgs::PoseArray parr;
        parr.header.frame_id = transform_samples[0].frame_id_;
        parr.header.stamp = ros::Time(0);
        for (std::vector<StampedTransform>::iterator it = transform_samples.begin(); it != transform_samples.end(); ++it)
        {
            geometry_msgs::Pose ps;
            tf::poseTFToMsg(*it,ps);
            parr.poses.push_back(ps);
        }

        if (1)
        {
            MatrixXd sample_covar;
            MatrixXd samples = ulistener.sampleSetTFtoMatrixXd(transform_samples);
            //ulistener.calculateSampleCovariance(samples.transpose(), samples.transpose(), sample_covar);
            ulistener.calculateSampleCovariance(samples, samples, sample_covar);
            std::cout << "sample covariance " << endl << sample_covar << endl;
            MatrixXd mean = ulistener.calculateSampleMean(samples);
            std::cout << "sample mean" << endl << mean << endl;
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

            //void sampleFromMeanCov(const tf::Transform &mean, const MatrixXd &cov, std::vector<tf::Transform> &output, size_t n = 1);
        }

        poseArrPub.publish(parr);

    }
    return 0;
};

