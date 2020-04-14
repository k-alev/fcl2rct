#pragma once

#include <utils_fcl.h>
#include <fcl/broadphase/broadphase_bruteforce.h>
#include <fcl/distance.h>
#include <ros_control_toolbox/status.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace fcl
{
class NaiveCollisionManagerVct : public NaiveCollisionManager
{
public:
  NaiveCollisionManagerVct() {}

  void distanceVct(CollisionObject *obj, std::vector<fcl::DistanceData> &cdata_, const double& max_dist, DistanceCallBack callback) const;

  void distanceVct(std::vector<std::vector<fcl::DistanceData>> &cdata_, const double& max_dist, DistanceCallBack callback) const;

  void distanceVct(BroadPhaseCollisionManager* other_manager_, std::vector<std::vector<fcl::DistanceData>> &cdata_, const double &max_dist,
  DistanceCallBack callback) const;

  void updateVct(const std::vector<rct::Status>& stata, const std::vector<fcl::Transform3f>& offsets);//const??

  void registerVct(const std::vector<fcl::CollisionObject*>& objs);//const??

  std::vector<rct::Status> removeDuplicates(const std::vector<rct::Status>& stata);

  void getDistanceVector(fcl::DistanceData* cdata, fcl::Transform3f tmpTfm1, fcl::Transform3f tmpTfm2) const;//const?? //should be private

  void distanceVct_Plus(BroadPhaseCollisionManager *other_manager_, std::vector<std::vector<fcl::DistanceData>> &cdata_,
  const double &max_dist, DistanceCallBack callback) const;
};

} // namespace fcl