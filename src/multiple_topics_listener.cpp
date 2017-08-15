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
  tf.setTranslation(Vec3f(object->transform.translation.x,
                          object->transform.translation.y,
                          object->transform.translation.z));
  tf.setQuatRotation(Quaternion3f(object->transform.rotation.x,
                                  object->transform.rotation.y,
                                  object->transform.rotation.z,
                                  object->transform.rotation.w));
  return tf;
}

vector<Contact> getPointingObject(const TransformStampedConstPtr& cube,
                                  const TransformStampedConstPtr& wand){
  shared_ptr<Box> cube_box(new Box(0.01, 0.01, 0.01));
  shared_ptr<Cylinder> wand_cylinder(new Cylinder(0.1, 10));

  GJKSolver_libccd solver;
  Vec3f contact_points;
  FCL_REAL penetration_depth;
  Vec3f normal;

  Transform3f tf_cube, tf_table, tf_wand;
  tf_cube = setTranformation(cube);
  tf_wand = setTranformation(wand);

  bool res = solver.shapeIntersect(*cube_box, tf_cube, *wand_cylinder, tf_wand,
                                   &contact_points, &penetration_depth, &normal);
  static const int num_max_contacts = std::numeric_limits<int>::max();
  static const bool enable_contact = true;
  CollisionResult result;
  CollisionRequest request(num_max_contacts, enable_contact);

  CollisionObject co0(cube_box, tf_cube);
  CollisionObject co1(wand_cylinder, tf_wand);

  collide(&co0, &co1, request, result);
  vector<Contact> contacts;
  result.getContacts(contacts);

  return contacts;
}
void callback(const TransformStampedConstPtr& cube_1,
                     const TransformStampedConstPtr& cube_2,
                     const TransformStampedConstPtr& cube_3,
                     const TransformStampedConstPtr& table,
                     const TransformStampedConstPtr& wand){

  vector<Contact> contacts_1 = getPointingObject(cube_1, wand);
  vector<Contact> contacts_2 = getPointingObject(cube_2, wand);
  vector<Contact> contacts_3 = getPointingObject(cube_3, wand);
  string solution = "";
  if (!contacts_1.empty()){
    solution = "Wand is pointing to Cube 1";
    std::cout << "Wand is pointing to Cube 1" << '\n';
    cout << contacts_1.size() << " contacts found" << endl;
    for(const Contact &contact : contacts_1) {
    cout << "position: " << contact.pos << endl;
    }
  }
  if (!contacts_2.empty()){
    solution = "Wand is pointing to Cube 2";
    std::cout << "Wand is pointing to Cube 2" << '\n';
    cout << contacts_2.size() << " contacts found" << endl;
    for(const Contact &contact : contacts_2) {
    cout << "position: " << contact.pos << endl;
    }
  }
  if (!contacts_3.empty()){
    solution = "Wand is pointing to Cube 3";
    std::cout << "Wand is pointing to Cube 3" << '\n';
    cout << contacts_3.size() << " contacts found" << endl;
    for(const Contact &contact : contacts_3) {
    cout << "position: " << contact.pos << endl;
    }
  }
  cout << '\n';
  cout << '\n';
  cout << '\n';

}

int main(int argc, char **argv){

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::master::V_TopicInfo master_topics;

  message_filters::Subscriber<TransformStamped> cube_subscriber_1(n, "vicon/Cube1/Cube1", 10);
  message_filters::Subscriber<TransformStamped> cube_subscriber_2(n, "vicon/Cube2/Cube2", 10);
  message_filters::Subscriber<TransformStamped> cube_subscriber_3(n, "vicon/Cube3/Cube3", 10);
  message_filters::Subscriber<TransformStamped> table_subscriber(n, "vicon/Table/Table", 10);
  message_filters::Subscriber<TransformStamped> wand_subscriber(n, "vicon/Wand/Wand", 10);

  typedef sync_policies::ApproximateTime<TransformStamped, TransformStamped, TransformStamped, TransformStamped, TransformStamped> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cube_subscriber_1, cube_subscriber_2, cube_subscriber_3, table_subscriber, wand_subscriber);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

  ros::spin();

  return 0;
}
