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


void NaiveCollisionManagerVct::updateVct(const std::vector<rct::Status>& statuser)
{
    if (statuser.size()!=objs.size())
        throw std::runtime_error("Links size and registered object size do not match.");

    auto p_obj = objs.begin();
    for(auto it = statuser.begin(); it != statuser.end(); ++it)
    {
        auto i = it - statuser.begin();
        fcl::Quaternion3f Q = fcl::Quaternion3f(it->quat.w(), it->quat.x(), it->quat.y(), it->quat.z());
        fcl::Vec3f T = fcl::Vec3f(it->frame.pos(0), it->frame.pos(1), it->frame.pos(2));
        fcl::Transform3f transform(Q, T);

        (*p_obj)->setTransform(transform);
        p_obj++;
    }
}

void NaiveCollisionManagerVct::registerVct(const std::vector<std::string> desc)//const??
{
    std::vector<KDL::Tree> trees;

    for (auto it = desc.begin(); it!=desc.end(); ++it)
    {
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(*it, tree))
           throw std::runtime_error("Failed to construct KDL tree.");
        trees.push_back(tree);
    }

    registerVct(trees);
}

void NaiveCollisionManagerVct::registerVct(const std::vector<KDL::Tree> trees)//const??
{
    auto seg = trees[0].getSegment("box");
    auto f = seg->first;
    std::cout<<"first: "<<f<<std::endl;
    auto s = seg->second;
    // s.segment.
}


} // namespace fcl
