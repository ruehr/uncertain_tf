

#include <uncertain_tf/CovarianceTimeCache.h>

using namespace Eigen;
using namespace tf;
using namespace uncertain_tf;
using namespace std;

CovarianceStorage::CovarianceStorage(const MatrixXd& data, ros::Time stamp)
//, CompactFrameID frame_id)
    : covariance_(data)
    , stamp_(stamp)
    //, frame_id_(frame_id)
{ }

CovarianceTimeCache::CovarianceTimeCache(ros::Duration max_storage_time)
    : max_storage_time_(max_storage_time)
{
}

bool CovarianceTimeCache::getData(ros::Time time, CovarianceStorage & data_out, std::string* error_str)
{
    CovarianceStorage* p_temp_1 = NULL;
    CovarianceStorage* p_temp_2 = NULL;

    int num_nodes = findClosest(p_temp_1, p_temp_2, time, error_str);
    if (num_nodes == 0)
    {
        return false;
    }
    else if (num_nodes == 1)
    {
        data_out = *p_temp_1;
    }
    else if (num_nodes == 2)
    {
        //if( p_temp_1->frame_id_ == p_temp_2->frame_id_)
        //{
            interpolate(*p_temp_1, *p_temp_2, time, data_out);
        //}
        //else
        //{
        //    data_out = *p_temp_1;
        //}
    }
    else
    {
        ROS_BREAK();
    }

    return true;
}

bool CovarianceTimeCache::insertData(const CovarianceStorage& new_data)
{
    //std::cout << "insertData new_data.stamp_ " << new_data.stamp_ << endl;

    L_CovarianceStorage::iterator storage_it = storage_.begin();

    if(storage_it != storage_.end())
    {
        if (storage_it->stamp_ > new_data.stamp_ + max_storage_time_)
        {
            std::cout << storage_it->stamp_ << " should not be bigger than " <<  new_data.stamp_  << " + " <<  max_storage_time_ << endl;
            return false;
        }
    }
    else
    {
        //std::cout << "storage empty" << endl;
    }

    while(storage_it != storage_.end())
    {
        if (storage_it->stamp_ <= new_data.stamp_)
            break;
        storage_it++;
    }
    storage_.insert(storage_it, new_data);

    //std::cout << "storage size " << storage_.size() << endl;

    pruneList();
    return true;
}


uint8_t CovarianceTimeCache::findClosest(CovarianceStorage*& one, CovarianceStorage*& two, ros::Time target_time, std::string* error_str)
{
    //No values stored
    if (storage_.empty())
    {
        //createEmptyException(error_str);
        return 0;
    }

    //If time == 0 return the latest
    if (target_time.isZero())
    {
        one = &storage_.front();
        return 1;
    }

    // One value stored
    if (++storage_.begin() == storage_.end())
    {
        CovarianceStorage& ts = *storage_.begin();
        if (ts.stamp_ == target_time)
        {
            one = &ts;
            return 1;
        }
        else
        {
            //createExtrapolationException1(target_time, ts.stamp_, error_str);
            return 0;
        }
    }

    ros::Time latest_time = (*storage_.begin()).stamp_;
    ros::Time earliest_time = (*(storage_.rbegin())).stamp_;

    if (target_time == latest_time)
    {
        one = &(*storage_.begin());
        return 1;
    }
    else if (target_time == earliest_time)
    {
        one = &(*storage_.rbegin());
        return 1;
    }
    // Catch cases that would require extrapolation
    else if (target_time > latest_time)
    {
        //createExtrapolationException2(target_time, latest_time, error_str);
        ROS_ERROR("EXTRAPOLATION TO FUTURE REQ");
        return 0;
    }
    else if (target_time < earliest_time)
    {
        //createExtrapolationException3(target_time, earliest_time, error_str);
        ROS_ERROR("EXTRAPOLATION TO PAST REQ");
        return 0;
    }

    //At least 2 values stored
    //Find the first value less than the target value
    L_CovarianceStorage::iterator storage_it = storage_.begin();
    while(storage_it != storage_.end())
    {
        if (storage_it->stamp_ <= target_time)
            break;
        storage_it++;
    }

    //Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
    one = &*(storage_it); //Older
    two = &*(--storage_it); //Newer
    return 2;

}

void CovarianceTimeCache::interpolate(const CovarianceStorage& one, const CovarianceStorage& two, ros::Time time, CovarianceStorage& output)
{
    output = two; //! TODO
}

void CovarianceTimeCache::pruneList()
{
    ros::Time latest_time = storage_.begin()->stamp_;

    while(!storage_.empty() && storage_.back().stamp_ + max_storage_time_ < latest_time)
    {
        storage_.pop_back();
    }
}
