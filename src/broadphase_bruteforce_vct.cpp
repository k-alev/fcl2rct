#include <broadphase_bruteforce_vct.h>

namespace fcl
{

void NaiveCollisionManagerVct::distanceVct(CollisionObject *obj, std::vector<fcl::DistanceData> &cdata_, const double &max_dist, DistanceCallBack callback) const
{
    if (size() == 0)
        return;

    fcl::DistanceData *cdata = new fcl::DistanceData;

    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    for (std::list<CollisionObject *>::const_iterator it = objs.begin(), end = objs.end(); it != end; ++it)
    {
        if (obj->getAABB().distance((*it)->getAABB()) < max_dist)
        {
            cdata->result.min_distance = max_dist;
            if (callback(obj, *it, cdata, min_dist))
            {
                cdata_.push_back(*cdata);
                return;
            }
            cdata_.push_back(*cdata);
        }
        else
        {
            cdata->result.min_distance = max_dist + 1;
            cdata_.push_back(*cdata);
        }
    }
}

void NaiveCollisionManagerVct::distanceVct(std::vector<std::vector<fcl::DistanceData>> &cdata_, const double &max_dist, DistanceCallBack callback) const
{
    if (size() == 0)
        return;

    fcl::DistanceData *cdata = new fcl::DistanceData;
    std::vector<fcl::DistanceData> cdata_tmp;

    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    for (std::list<CollisionObject *>::const_iterator it1 = objs.begin(), end = objs.end(); it1 != end; ++it1)
    {
        cdata_tmp.clear();
        std::list<CollisionObject *>::const_iterator it2 = it1;
        it2++;
        for (; it2 != end; ++it2)
        {
            cdata->result.min_distance = max_dist;
            if ((*it1)->getAABB().distance((*it2)->getAABB()) < max_dist)
            {
                if (callback(*it1, *it2, cdata, min_dist))
                {
                    cdata_tmp.push_back(*cdata);
                    return;
                }
                cdata_tmp.push_back(*cdata);
            }
            else
            {
                cdata->result.min_distance = max_dist + 1;
                cdata_tmp.push_back(*cdata);
            }
        }
        cdata_.push_back(cdata_tmp);
    }
}

void NaiveCollisionManagerVct::distanceVct(BroadPhaseCollisionManager *other_manager_, std::vector<std::vector<fcl::DistanceData>> &cdata_,
const double &max_dist, DistanceCallBack callback) const
{
    NaiveCollisionManager *other_manager = static_cast<NaiveCollisionManager *>(other_manager_);

    if ((size() == 0) || (other_manager->size() == 0))
        return;

      if(this == other_manager)
      {
        distanceVct(cdata_, max_dist, callback);
        return;
      }

    fcl::DistanceData *cdata = new fcl::DistanceData;
    std::vector<fcl::DistanceData> cdata_tmp;
    std::vector<CollisionObject *> other_objs;
    other_manager->getObjects(other_objs);

    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    for (std::list<CollisionObject *>::const_iterator it1 = objs.begin(), end1 = objs.end(); it1 != end1; ++it1)
    {
        cdata_tmp.clear();
        // for (std::list<CollisionObject *>::const_iterator it2 = other_manager->objs.begin(), end2 = other_manager->objs.end(); it2 != end2; ++it2)
        for (std::vector<CollisionObject *>::const_iterator it2 = other_objs.begin(), end2 = other_objs.end(); it2 != end2; ++it2)
        {
            cdata->result.min_distance = max_dist;
            if ((*it1)->getAABB().distance((*it2)->getAABB()) < max_dist)
            {
                if (callback(*it1, *it2, cdata, min_dist))
                {
                    cdata_tmp.push_back(*cdata);
                    return;
                }
                cdata_tmp.push_back(*cdata);
            }
            else
            {
                cdata->result.min_distance = max_dist + 1;
                cdata_tmp.push_back(*cdata);
            }
        }
        cdata_.push_back(cdata_tmp);
    }
}


void NaiveCollisionManagerVct::updateVct(const std::vector<rct::Status>& stata, const std::vector<fcl::Transform3f>& offsets)
{
    if (stata.size()!=objs.size())
    {
        std::cout<<"SizeStata: "<<stata.size()<< std::endl;
        std::cout<<"SizeObj: "<<objs.size()<< std::endl;
        throw std::runtime_error("Links number and registered objects number do not match.");
    }

    auto p_obj = objs.begin();
    auto p_off = offsets.begin();
    for(auto it = stata.begin(); it != stata.end(); ++it)
    {
        auto i = it - stata.begin();
        fcl::Quaternion3f Q = fcl::Quaternion3f(it->quat.w(), it->quat.x(), it->quat.y(), it->quat.z());
        fcl::Vec3f T = fcl::Vec3f(it->frame.pos(0), it->frame.pos(1), it->frame.pos(2));
        fcl::Transform3f transform(Q, T);

        std::cout<<"Pos Before Translation: "<<T.data[0]<<", "<<T.data[1]<<", "<<T.data[2]<< std::endl;
        auto takis = (transform*(*p_off)).getTranslation();
        std::cout<<"Pos After Translation: "<<takis.data[0]<<", "<<takis.data[1]<<", "<<takis.data[2]<< std::endl;

        (*p_obj)->setTransform(transform*(*p_off));
        p_obj++;
        p_off++;
    }
}

void NaiveCollisionManagerVct::registerVct(const std::vector<fcl::CollisionObject*>& objs)//const??
{
    this->registerObjects(objs);
}

std::vector<rct::Status> NaiveCollisionManagerVct::removeDuplicates(const std::vector<rct::Status>& stata) 
{
    // A really bad way to remove dummy links(without collision) inside the chain.
    std::vector<rct::Status> novaStata;
    for(auto it = std::next(stata.begin(),1); it != stata.end(); ++it)
    {
        auto it_prv = std::prev(it,1);
        if((it->frame.pos[0] != it_prv->frame.pos[0])||(it->frame.pos[1] != it_prv->frame.pos[1])||(it->frame.pos[2] != it_prv->frame.pos[2]))
        {
            novaStata.push_back(*it_prv);
        }
    }
    auto it = std::prev(stata.end(),1);
    novaStata.push_back(*it);
    return novaStata;
}

void NaiveCollisionManagerVct::getDistanceVector(fcl::DistanceData* cdata, fcl::Transform3f tmpTfm1, fcl::Transform3f tmpTfm2) const //should be private
{
    //returns distance vector between the two nearest points expressed in the world frame
    fcl::Transform3f npTfm1(cdata->result.nearest_points[0]);
    fcl::Transform3f npTfm2(cdata->result.nearest_points[1]);
    auto tmp = (tmpTfm1*npTfm1).getTranslation();
    cdata->distVctWF.first = (tmpTfm2*npTfm2).getTranslation() - tmp;
    cdata->distVctWF.second = tmp;
}

void NaiveCollisionManagerVct::distanceVct_Plus(BroadPhaseCollisionManager *other_manager_, std::vector<std::vector<fcl::DistanceData>> &cdata_,
const double &max_dist, DistanceCallBack callback) const
{
    NaiveCollisionManager *other_manager = static_cast<NaiveCollisionManager *>(other_manager_);

    if ((size() == 0) || (other_manager->size() == 0))
        return;
    if(this == other_manager)
      {
        distanceVct(cdata_, max_dist, callback);
        return;
      }
    fcl::DistanceData *cdata = new fcl::DistanceData;
    std::vector<fcl::DistanceData> cdata_tmp;
    std::vector<CollisionObject *> other_objs;
    other_manager->getObjects(other_objs);

    FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
    for (std::list<CollisionObject *>::const_iterator it1 = objs.begin(), end1 = objs.end(); it1 != end1; ++it1)
    {
        auto tmpTfm1 = (*it1)->getTransform();
        cdata_tmp.clear();
        for (std::vector<CollisionObject *>::const_iterator it2 = other_objs.begin(), end2 = other_objs.end(); it2 != end2; ++it2)
        {
            auto tmpTfm2 = (*it2)->getTransform();
            cdata->result.min_distance = max_dist;
            if ((*it1)->getAABB().distance((*it2)->getAABB()) < max_dist)
            {
                if (callback(*it1, *it2, cdata, min_dist))
                {
                    getDistanceVector(cdata, tmpTfm1, tmpTfm2);
                    // std::cout<<"DISTANCE NORM: "<<cdata->distVctWF.first.length()<<std::endl;
                    // std::cout<<"Fist Vec: "<<cdata->distVctWF.second<<std::endl;
                    cdata_tmp.push_back(*cdata);
                    return;
                }
                getDistanceVector(cdata, tmpTfm1, tmpTfm2);
                // std::cout<<"DISTANCE NORM: "<<cdata->distVctWF.first.length()<<std::endl;
                // std::cout<<"Fist Vec: "<<cdata->distVctWF.second<<std::endl;
                cdata_tmp.push_back(*cdata);
            }
            else
            {   
                fcl::Vec3f tmpVct = fcl::Vec3f();
                cdata->distVctWF.first = tmpVct;
                cdata->distVctWF.second = tmpVct;
                cdata->result.min_distance = max_dist + 1;
                cdata_tmp.push_back(*cdata);
            }
        }
        cdata_.push_back(cdata_tmp);
    }
}

} // namespace fcl
