#pragma once

#include "fcl/config.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"
#include <fcl/collision_data.h>
#include "fcl/distance.h"

#include <iomanip>
#include <utils.h>
#include <broadphase_bruteforce_vct.h>


class FCLdistance2
{
public:
FCLdistance2(){};
~FCLdistance2(){};

void generateEnvironment(std::vector<fcl::CollisionObject *> &env);
void generateRobot(std::vector<fcl::CollisionObject *> &rbt);
// void generateEnvironmentMesh(std::vector<fcl::CollisionObject *> &rbt);
// void generateRobotMesh(std::vector<fcl::CollisionObject *> &rbt);

void broadphase_distance();
void broadphase_distance_vct();

//sceleton


};
