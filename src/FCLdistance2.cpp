#include <FCLdistance2.h>

void FCLdistance2::generateEnvironment(std::vector<fcl::CollisionObject *> &env)
{
    double box_size = 1;
    fcl::Box *box = new fcl::Box(box_size, box_size, box_size);
    for (int i = 0; i < 1; i++)
    {
        env.push_back(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),
                                               fcl::Transform3f(fcl::Vec3f(0, 0, 0))));

        env.push_back(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),
                                               fcl::Transform3f(fcl::Vec3f(50, 20, 10))));

        env.push_back(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),
                                               fcl::Transform3f(fcl::Vec3f(10, 0, 0))));

        // env.push_back(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),
        //                                        fcl::Transform3f(fcl::Vec3f(200, 0, 0))));
    }
}

void FCLdistance2::generateRobot(std::vector<fcl::CollisionObject *> &rbt)
{
    double box_size = 1;
    fcl::Box *box = new fcl::Box(box_size, box_size, box_size);
    for (int i = 0; i < 1; i++)
    {
        rbt.push_back(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),
                                               fcl::Transform3f(fcl::Vec3f(100, 0, 0))));

        rbt.push_back(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),
                                               fcl::Transform3f(fcl::Vec3f(140, 0, 0))));

        rbt.push_back(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(box),
                                               fcl::Transform3f(fcl::Vec3f(70, 0, 0))));
    }
}

void FCLdistance2::broadphase_distance()
{
    std::vector<fcl::CollisionObject *> env;
    std::vector<fcl::CollisionObject *> rbt;
    generateEnvironment(env);
    generateRobot(rbt);

    std::vector<fcl::BroadPhaseCollisionManager *> managers;

    managers.push_back(new fcl::NaiveCollisionManager());
    managers.push_back(new fcl::NaiveCollisionManager());

    managers[0]->registerObjects(env);
    managers[1]->registerObjects(rbt);

    managers[0]->setup();
    managers[1]->setup();

    std::vector<fcl::DistanceData> self_data(managers.size());
    managers[0]->distance(&self_data[0], fcl::defaultDistanceFunction);
    managers[1]->distance(&self_data[1], fcl::defaultDistanceFunction);

    // self_data[0].request();
    std::cout << "0: dist: " << self_data[0].result.min_distance << std::endl;
    std::cout << "1: dist: " << self_data[1].result.min_distance << std::endl;
    // std::cout<<"0: n points: "<<self_data[0].result.nearest_points<<std::endl;
    // std::cout<<"0: n points: "<<self_data[1].result.nearest_points<<std::endl;

    fcl::DistanceData datum;
    std::vector<fcl::CollisionObject *> objs;
    managers[0]->getObjects(objs);
    managers[1]->distance(objs[0], &datum, fcl::defaultDistanceFunction);
    std::cout << "2: dist: " << datum.result.min_distance << std::endl;

    fcl::DistanceData datum2;
    managers[0]->distance(managers[1], &datum2, fcl::defaultDistanceFunction);
    std::cout << "3: dist: " << datum2.result.min_distance << std::endl;
    datum.request.enable_nearest_points = 1;
}

void FCLdistance2::broadphase_distance_vct()
{
    std::vector<fcl::CollisionObject *> env;
    std::vector<fcl::CollisionObject *> rbt;
    generateEnvironment(env);
    generateRobot(rbt);

    std::vector<fcl::BroadPhaseCollisionManager *> managers;

    managers.push_back(new fcl::NaiveCollisionManager());
    managers.push_back(new fcl::NaiveCollisionManagerVct());

    managers[0]->registerObjects(env);
    managers[1]->registerObjects(rbt);

    managers[0]->setup();
    managers[1]->setup();

    // std::vector<fcl::DistanceData> self_data(managers.size());
    // managers[0]->distance(&self_data[0], fcl::defaultDistanceFunction);
    // managers[1]->distance(&self_data[1], fcl::defaultDistanceFunction);

    // // self_data[0].request();
    // std::cout << "0: dist: " << self_data[0].result.min_distance << std::endl;
    // std::cout << "1: dist: " << self_data[1].result.min_distance << std::endl;
    // // std::cout<<"0: n points: "<<self_data[0].result.nearest_points<<std::endl;
    // // std::cout<<"0: n points: "<<self_data[1].result.nearest_points<<std::endl;

    // fcl::DistanceData datum;
    // std::vector<fcl::CollisionObject *> objs;
    // managers[0]->getObjects(objs);
    // managers[1]->distance(objs[0], &datum, fcl::defaultDistanceFunction);
    // std::cout << "2: dist: " << datum.result.min_distance << std::endl;

    // fcl::DistanceData datum2;
    // managers[0]->distance(managers[1], &datum2, fcl::defaultDistanceFunction);
    // std::cout << "3: dist: " << datum2.result.min_distance << std::endl;
    // datum2.request.enable_nearest_points = 1;

    // std::vector<fcl::DistanceData> *self_data= new std::vector<fcl::DistanceData>;
    // std::vector<fcl::CollisionObject *> objs;
    // managers[0]->getObjects(objs);
    // managers[1]->distance(objs[0], self_data, fcl::defaultDistanceFunction);

    // std::cout << "self_data ptr: " <<self_data<< std::endl;

    // for(std::vector<fcl::DistanceData>::iterator it = self_data->begin(); it!= self_data->end(); ++it)
    // {
    //     std::cout<<"iter: "<<(it++)->result.min_distance<<std::endl;
    // }

    ////////////////////////////////////////////////////////////////////////////////////
    fcl::NaiveCollisionManagerVct *managerNuevo = new fcl::NaiveCollisionManagerVct;

    managerNuevo->registerObjects(env);
    // managerNuevo2->registerObjects(rbt);

    managerNuevo->setup();

    std::vector<fcl::DistanceData> self_data;
    std::vector<fcl::CollisionObject *> objs;
    managers[1]->getObjects(objs);
    // managerNuevo->distanceVct(objs[0], self_data, fcl::defaultDistanceFunction);
    managerNuevo->distanceVct(objs[0], self_data, 100, fcl::defaultDistanceFunction);

    std::cout << "size: " << self_data.size() << std::endl;
    std::cout << "dist: " << self_data[0].result.min_distance << std::endl;

    for (std::vector<fcl::DistanceData>::iterator it = self_data.begin(); it != self_data.end(); ++it)
    {
        std::cout << "iter: " << (it)->result.min_distance << std::endl;
    }

    ///////////////////////////////////

    // std::vector<fcl::DistanceData> self_data2;
    // managerNuevo->distanceVct(self_data2, 300, fcl::defaultDistanceFunction);

    // std::cout<<"size: "<<self_data2.size()<<std::endl;
    // for(std::vector<fcl::DistanceData>::iterator it = self_data2.begin(); it!= self_data2.end(); ++it)
    // {
    //     std::cout<<"iter2: "<<(it)->result.min_distance<<std::endl;
    // }

    //////////////////////////////////////////////
    std::vector<std::vector<fcl::DistanceData>> self_data3;
    managerNuevo->distanceVct(self_data3, 300, fcl::defaultDistanceFunction);

    for (int i = 0; i < self_data3.size(); ++i)
    {
        for (std::vector<fcl::DistanceData>::iterator it = self_data3[i].begin(); it != self_data3[i].end(); ++it)
        {
            std::cout << "fiter" << i << ": " << (it)->result.min_distance << std::endl;
        }
    }

    ///////////////////////////////////////////////////

    std::vector<std::vector<fcl::DistanceData>> datum2;
    fcl::Timer timer;
    timer.start();
    managerNuevo->distanceVct(managers[1], datum2, 300, fcl::defaultDistanceFunction);
    timer.stop();

    std::cout << "size: " << datum2.size() << std::endl;
    for (int i = 0; i < datum2.size(); ++i)
    {
        for (std::vector<fcl::DistanceData>::iterator it = datum2[i].begin(); it != datum2[i].end(); ++it)
        {
            std::cout << "other_mngr" << i << ": " << (it)->result.min_distance << std::endl;
            std::cout << "other_mngr_1pts" << i << ": " << (it)->result.nearest_points[0] << std::endl;
            std::cout << "other_mngr_2pts" << i << ": " << (it)->result.nearest_points[1] << std::endl;
        }
    }

    std::cout << "Elapsed Time: " << timer.getElapsedTimeInSec() << std::endl;

    /////////////////////////////////////////////////////

    std::vector<fcl::CollisionObject*> env2;
    fcl::Ellipsoid *ellipsoid = new fcl::Ellipsoid(1, 2, 2);
    env2.push_back(new fcl::CollisionObject(std::shared_ptr<fcl::CollisionGeometry>(ellipsoid),
                                           fcl::Transform3f(fcl::Vec3f(1, 2, 1))));


    fcl::Matrix3f R;
    R.setValue(1, 2, 3, 4, 5, 6, 7, 8, 9);
    fcl::Vec3f T(1, 2, 3);
    // fcl::Vec3f T2(5, 6, 7);
    // fcl::Transform3f t1(R, T);
    // fcl::Transform3f t2(R, T);
    // t2.setTransform(R, T2);
    fcl::Transform3f trsf =  env2[0]->getTransform();
    env2[0]->setTranslation(T);
    fcl::Transform3f trsf2 =  env2[0]->getTransform();

    std::cout<<trsf.getTranslation()<<std::endl;
    std::cout<<trsf2.getTranslation()<<std::endl;

    ///////////////////////////////////////////////////C
    std::vector<rct::Status> links;
    rct::Status link1;
    rct::Status link2;
    rct::Status link3;
    link1.frame.pos(0) = 10;
    link2.frame.pos(1) = 20;
    link3.frame.pos(2) = 30;
    links.push_back(link1);
    links.push_back(link2);
    links.push_back(link3);
    managerNuevo->updateVct(links);
    std::cout<<"Laxana poulaw"<<std::endl;
    std::cout<<env[0]->getTranslation()<<std::endl;
    std::cout<<env[1]->getTranslation()<<std::endl;
    std::cout<<env[2]->getTranslation()<<std::endl;
    



}
