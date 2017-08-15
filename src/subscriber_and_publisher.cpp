#include <ros/ros.h>
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

typedef message_filters::sync_policies::ApproximateTime<TransformStamped,
                                                        TransformStamped,
                                                        TransformStamped,
                                                        TransformStamped,
                                                        TransformStamped>
                                                        MySyncPolicy;
typedef message_filters::Subscriber<TransformStamped> TransformStampedType;

class SubscribeAndPublish{

public:
  SubscribeAndPublish(){
    publisher = n.advertise<std_msgs::String>("/alexa_out", 1);
    cube_subscriber_1 = new TransformStampedType(n, "vicon/Cube1/Cube1", 10);
    cube_subscriber_2 = new TransformStampedType(n, "vicon/Cube2/Cube2", 10);
    cube_subscriber_3 = new TransformStampedType(n, "vicon/Cube3/Cube3", 10);
    table_subscriber = new TransformStampedType(n, "vicon/Table/Table", 10);
    wand_subscriber = new TransformStampedType(n, "vicon/Wand/Wand", 10);

    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),
                                                          *cube_subscriber_1,
                                                          *cube_subscriber_2,
                                                          *cube_subscriber_3,
                                                          *table_subscriber,
                                                          *wand_subscriber);
    sync->registerCallback(boost::bind(&SubscribeAndPublish::callback, this,
                                                  _1, _2, _3, _4, _5));
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
    std_msgs::String msg;

    if (!contacts_1.empty()){
      solution = "Wand is pointing to Cube 1";
      cout << "Wand is pointing to Cube 1" << endl;
      cout << contacts_1.size() << " contacts found" << endl;
      for(const Contact &contact : contacts_1) {
      cout << "position: " << contact.pos << endl;
      }
    }
    if (!contacts_2.empty()){
      solution = "Wand is pointing to Cube 2";
      cout << "Wand is pointing to Cube 2" << endl;
      cout << contacts_2.size() << " contacts found" << endl;
      for(const Contact &contact : contacts_2) {
      cout << "position: " << contact.pos << endl;
      }
    }
    if (!contacts_3.empty()){
      solution = "Wand is pointing to Cube 3";
      cout << "Wand is pointing to Cube 3" << endl;
      cout << contacts_3.size() << " contacts found" << endl;
      for(const Contact &contact : contacts_3) {
      cout << "position: " << contact.pos << endl;
      }
    }
    cout << '\n';
    cout << '\n';
    cout << '\n';
    msg.data = solution;
    publisher.publish(msg);

  }

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
//private:
  ros::NodeHandle n;
  ros::Publisher publisher;
  message_filters::Subscriber<TransformStamped> *cube_subscriber_1;
  message_filters::Subscriber<TransformStamped> *cube_subscriber_2;
  message_filters::Subscriber<TransformStamped> *cube_subscriber_3;
  message_filters::Subscriber<TransformStamped> *table_subscriber;
  message_filters::Subscriber<TransformStamped> *wand_subscriber;
  message_filters::Synchronizer<MySyncPolicy>* sync;
};

int main(int argc, char **argv){

  ros::init(argc, argv, "subscribe_and_publish");
  SubscribeAndPublish SAPObject;
  ros::Rate r(0.5);
  while (true){
    ros::spinOnce();
    r.sleep();
   }
  return 0;
}
