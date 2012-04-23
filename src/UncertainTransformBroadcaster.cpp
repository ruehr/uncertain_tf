
#include "uncertain_tf/UncertainTransformBroadcaster.h"
#include "uncertain_tf/utfMessage.h"


using namespace Eigen;
using namespace tf;
using namespace uncertain_tf;
using namespace std;

namespace uncertain_tf {


UncertainTransformBroadcaster::UncertainTransformBroadcaster()
{
    publisher_ = node_.advertise<uncertain_tf::utfMessage>("/tf_uncertainty", 100);
    ros::NodeHandle l_nh("~");
    //tf_prefix_ = getPrefixParam(l_nh);
}

void UncertainTransformBroadcaster::sendCovariance(const StampedCovariance& covariance)
{
    std::vector<StampedCovariance> v1;
    v1.push_back(covariance);
    sendCovariance(v1);
}

void UncertainTransformBroadcaster::sendCovariance(const std::vector<StampedCovariance>& covariances)
{
    std::vector<uncertain_tf::CovarianceStamped> utfm;
    for (std::vector<StampedCovariance>::const_iterator it = covariances.begin(); it != covariances.end(); ++it)
    {
        uncertain_tf::CovarianceStamped msg;
        covarianceStampedTFToMsg(*it, msg);
        utfm.push_back(msg);
    }
    sendCovariance(utfm);
}

void UncertainTransformBroadcaster::sendCovariance(const std::vector<CovarianceStamped>& covariances_msg)
{
    uncertain_tf::utfMessage msg;
    msg.covariances = covariances_msg;
    //std::cout << "publishing:" << endl << msg << endl;
    publisher_.publish(msg);
}

}
