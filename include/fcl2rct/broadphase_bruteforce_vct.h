#pragma once

#include <utils.h>
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

  // void distance(CollisionObject* obj, void* cdata, DistanceCallBack callback) const;

  // void distance(void* cdata, DistanceCallBack callback) const;

  // void distance(BroadPhaseCollisionManager* other_manager, void* cdata, DistanceCallBack callback) const;

  void distanceVct(CollisionObject *obj, std::vector<fcl::DistanceData> &cdata_, const double& max_dist, DistanceCallBack callback) const;

  void distanceVct(std::vector<std::vector<fcl::DistanceData>> &cdata_, const double& max_dist, DistanceCallBack callback) const;

  void distanceVct(BroadPhaseCollisionManager* other_manager_, std::vector<std::vector<fcl::DistanceData>> &cdata_, const double &max_dist,
  DistanceCallBack callback) const;

  void updateVct(const std::vector<rct::Status>& chain);//const??

  void registerVct(const std::vector<std::string> desc);//const??

  void registerVct(const std::vector<KDL::Tree> trees);//const??

};

} // namespace fcl