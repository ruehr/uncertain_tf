

#include "uncertain_tf/UncertainTransformer.h"


using namespace Eigen;
using namespace tf;
using namespace uncertain_tf;
using namespace std;

namespace uncertain_tf {


CovarianceTimeCache* UncertainTransformer::getCovariance(unsigned int frame_id)
{
    if (frame_id == 0) /// @todo check larger values too
        return NULL;
    else
    {
        while (frame_id >= covariances_.size())
            covariances_.push_back(NULL); // thats a hack

        // insert 0 covariance at time 0 to have covariances defined for all frames we query
        if (covariances_[frame_id] == NULL)
        {
            std::cout << "creating new CovarianceTimeCache for id " << frame_id << endl;
            covariances_[frame_id] = new CovarianceTimeCache(ros::Duration(10)); //!TODO : set actual max storage time
            MatrixXd cov(6,6);
            cov.setZero();
            covariances_[frame_id]->insertData(CovarianceStorage(cov, ros::Time(0))); //, frame_id));
        }
        return covariances_[frame_id];
    }
};

bool UncertainTransformer::setCovariance(const StampedCovariance &cov)
{
    std::string frame_id = tf::resolve(getTFPrefix(), cov.frame_id_);
    std::cout << "setCovariance " << frame_id << endl;
    // todo: set the parent id, we could run into cases where the parent changes and we still have the covariance for another parent

    StampedCovariance mapped_covariance((MatrixXd)cov, cov.stamp_, cov.frame_id_);
    mapped_covariance.frame_id_ = tf::resolve(getTFPrefix(), cov.frame_id_);

    {
        boost::recursive_mutex::scoped_lock lock(cov_mutex_);
        CompactFrameID frame_number = lookupOrInsertFrameNumber(mapped_covariance.frame_id_); //! this is different from the standard tf where we keep a vector of [child_frame_id]

        CovarianceTimeCache* covariance = getCovariance(frame_number);
        /*if (covariance == NULL)
        {
            std::cout << "creating new CovarianceTimeCache for id " << frame_number << endl;
            covariances_[frame_number] = new CovarianceTimeCache(cache_time);
            covariance = covariances_[frame_number];
        }*/

        if (!covariance->insertData(CovarianceStorage(mapped_covariance, cov.stamp_))) // ,lookupOrInsertFrameNumber(mapped_covariance.frame_id_))))
        {
            ROS_INFO("ERROR in cov-insert");
            return false;
        }

    }

    return true;
}


} // namespace uncertain_tf
