#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <fcl/shape/geometric_shapes.h>
#include "fcl/BVH/BV_fitter.h"
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include <iostream>
#include <fcl/collision.h>
#include<fcl/math/transform.h>

using namespace std;
using namespace fcl;
using namespace message_filters;
using namespace geometry_msgs;

Transform3f setTranformation(const TransformStampedConstPtr& object){
  Transform3f tf;
  tf.setIdentity();
  tf.setTranslation(Vec3f(object->transform.translation.x, object->transform.translation.y, object->transform.translation.z));
  tf.setQuatRotation(Quaternion3f(object->transform.rotation.x, object->transform.rotation.y, object->transform.rotation.z, object->transform.rotation.w));
  return tf;
}

Vec3f getPointingObject(const TransformStampedConstPtr& cube, const TransformStampedConstPtr& table, const TransformStampedConstPtr& wand){
  std::shared_ptr<Box> cube_box(new Box(0.001, 0.001, 0.001));
  std::shared_ptr<Box> table_box(new Box(0.5, 1, 0.001));
  std::shared_ptr<Cylinder> wand_cylinder(new Cylinder(0.2, 10));

  GJKSolver_libccd solver;
  Vec3f contact_points;
  FCL_REAL penetration_depth;
  Vec3f normal;

  Transform3f tf_cube, tf_table, tf_wand;
  tf_cube = setTranformation(cube);
  tf_table = setTranformation(table);
  tf_wand = setTranformation(wand);

  bool res = solver.shapeIntersect(*cube_box, tf_cube, *wand_cylinder, tf_wand, &contact_points, &penetration_depth, &normal);

  cout << "contact points: " << contact_points << endl;
  cout << "pen depth: " << penetration_depth << endl;
  cout << "normal: " << normal << endl;
  cout << "result: " << res << endl;
  if (res == true)  {
    cout << "BINGO" << '\n';
  }
  else  {
    cout << "NO BINGO" << '\n';
  }
    return contact_points;
}


void chatterCallback(const TransformStampedConstPtr& cube, const TransformStampedConstPtr& table, const TransformStampedConstPtr& wand){

  ROS_INFO_STREAM("***************** BEGIN ********************");
  ROS_INFO_STREAM("I heard CUBE: x=" << cube->transform.translation.x << " y = " << cube->transform.translation.y <<" z = " << cube->transform.translation.z);
  ROS_INFO_STREAM("I heard TABLE: x=" <<  table->transform.translation.x <<" y = " << table->transform.translation.y <<" z = " << table->transform.translation.z);
  ROS_INFO_STREAM("I heard WAND: x=" << wand->transform.translation.x << " y = " << wand->transform.translation.y <<" z = " << wand->transform.translation.z);
  ROS_INFO_STREAM("****************** END *********************");

  Vec3f v = getPointingObject(cube, table, wand);
  cout << '\n';
  cout << '\n';
  cout << '\n';
  cout << '\n';
}

int main(int argc, char **argv){

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::master::V_TopicInfo master_topics;
  message_filters::Subscriber<TransformStamped> cube_sub(n, "vicon/Cube/Cube", 10);
  message_filters::Subscriber<TransformStamped> table_sub(n, "vicon/Table/Table", 10);
  message_filters::Subscriber<TransformStamped> wand_sub(n, "vicon/Wand/Wand", 10);
  typedef sync_policies::ApproximateTime<TransformStamped, TransformStamped, TransformStamped> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cube_sub, table_sub, wand_sub);
  sync.registerCallback(boost::bind(&chatterCallback, _1, _2, _3));

  ros::spin();

  return 0;
}
