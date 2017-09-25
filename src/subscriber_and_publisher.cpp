#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <typeinfo>
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
                                                        TransformStamped,
                                                        TransformStamped,
                                                        TransformStamped>
                                                        MySyncPolicy;
typedef message_filters::Subscriber<TransformStamped> TransformStampedType;

class SubscribeAndPublish{

public:
  SubscribeAndPublish(){
    publisher_alexa = n.advertise<std_msgs::String>("/alexa_out", 1);
    coordinates_publisher = n.advertise<std_msgs::Float64MultiArray> ("/coordinates", 5);
    cube_subscriber_1 = new TransformStampedType(n, "vicon/Cube1/Cube1", 10);
    cube_subscriber_2 = new TransformStampedType(n, "vicon/Cube2/Cube2", 10);
    plate_subscriber = new TransformStampedType(n, "vicon/Plate/Plate", 10);
    banana_subscriber = new TransformStampedType(n, "vicon/Banana/Banana", 10);
    coffee_subscriber = new TransformStampedType(n, "vicon/Coffee/Coffee", 10);
    table_subscriber = new TransformStampedType(n, "vicon/Table/Table", 10);
    wand_subscriber = new TransformStampedType(n, "vicon/Wand/Wand", 10);


    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),
                                                          *cube_subscriber_1,
                                                          *cube_subscriber_2,
                                                          *plate_subscriber,
                                                          *banana_subscriber,
                                                          *coffee_subscriber,
                                                          *table_subscriber,
                                                          *wand_subscriber);
    sync->registerCallback(boost::bind(&SubscribeAndPublish::callback, this,
                                                  _1, _2, _3, _4, _5, _6, _7));
  }

  void callback(const TransformStampedConstPtr& cube_1,
                const TransformStampedConstPtr& cube_2,
                const TransformStampedConstPtr& plate,
                const TransformStampedConstPtr& banana,
                const TransformStampedConstPtr& coffee,
                const TransformStampedConstPtr& table,
                const TransformStampedConstPtr& wand){

    vector<Contact> contacts_1 = getPointingObject(cube_1, wand);
    vector<Contact> contacts_2 = getPointingObject(cube_2, wand);
    vector<Contact> contacts_3 = getPointingObject(plate, wand);
    vector<Contact> contacts_4 = getPointingObject(banana, wand);
    vector<Contact> contacts_5 = getPointingObject(coffee, wand);

    string solution = "";
    std_msgs::Float64MultiArray coordinates;
    coordinates.layout.dim.push_back(std_msgs::MultiArrayDimension());
    coordinates.layout.dim.push_back(std_msgs::MultiArrayDimension());
    coordinates.layout.dim.push_back(std_msgs::MultiArrayDimension());

    coordinates.layout.dim[0].size = 2;
    coordinates.layout.dim[0].stride = 2*5*3;
    coordinates.layout.dim[0].label = "index";

    coordinates.layout.dim[1].size = 5;
    coordinates.layout.dim[1].stride = 5*3;
    coordinates.layout.dim[1].label = "layer";

    coordinates.layout.dim[2].size = 3;
    coordinates.layout.dim[2].stride = 3;
    coordinates.layout.dim[2].label = "xyz";

    coordinates.data.clear();


    std_msgs::String msg;
    if (!contacts_1.empty()){
      solution = "Wand is pointing to Cube 1";

      cout << "Wand is pointing to Cube 1" << endl;
      cout << contacts_1.size() << " contacts found" << endl;
      for(const Contact &contact : contacts_1) {
        cout << "position: " << contact.pos << endl;
        Vec3f diff = get_difference(contact.pos, cube_1);
        for (int i=0; i < 3; i++)
        coordinates.data.insert(coordinates.data.end(), diff[i]);
      }
    }
    else{
      for (int i=0; i < 3; i++)
      coordinates.data.insert(coordinates.data.end(), 10);
    }

    if (!contacts_2.empty()){
      solution = "Wand is pointing to Cube 2";
      cout << "Wand is pointing to Cube 2" << endl;
      cout << contacts_2.size() << " contacts found" << endl;
      for(const Contact &contact : contacts_2) {
      cout << "position: " << contact.pos << endl;
      Vec3f diff = get_difference(contact.pos, cube_2);
      for (int i=0; i < 3; i++)
      coordinates.data.insert(coordinates.data.end(), diff[i]);
      }
    }
    else{
      for (int i=0; i < 3; i++)
      coordinates.data.insert(coordinates.data.end(), 10);
    }
    if (!contacts_3.empty()){
      solution = "Wand is pointing to Plate";
      cout << "Wand is pointing to Plate" << endl;
      cout << contacts_3.size() << " contacts found" << endl;
      for(const Contact &contact : contacts_3) {
      cout << "position: " << contact.pos << endl;
      Vec3f diff = get_difference(contact.pos, plate);
      for (int i=0; i < 3; i++)
      coordinates.data.insert(coordinates.data.end(), diff[i]);
      }
    }
    else{
      for (int i=0; i < 3; i++)
      coordinates.data.insert(coordinates.data.end(), 10);
    }
    if (!contacts_4.empty()){
      solution = "Wand is pointing to Banana";
      cout << "Wand is pointing to Banana" << endl;
      cout << contacts_4.size() << " contacts found" << endl;
      for(const Contact &contact : contacts_4) {
      cout << "position: " << contact.pos << endl;
      Vec3f diff = get_difference(contact.pos, banana);
      for (int i=0; i < 3; i++)
      coordinates.data.insert(coordinates.data.end(), diff[i]);
      }
    }
    else{
      for (int i=0; i < 3; i++)
      coordinates.data.insert(coordinates.data.end(), 10);
    }
    if (!contacts_5.empty()){
      solution = "Wand is pointing to Coffee";
      cout << "Wand is pointing to Coffee" << endl;
      cout << contacts_5.size() << " contacts found" << endl;
      for(const Contact &contact : contacts_5){
      cout << "position: " << contact.pos << endl;
      Vec3f diff = get_difference(contact.pos, coffee);
      for (int i=0; i < 3; i++)
      coordinates.data.insert(coordinates.data.end(), diff[i]);
      }
    }
    else{
      for (int i=0; i < 3; i++)
      coordinates.data.insert(coordinates.data.end(), 10);
    }

    Vec3f coor = get_coordinates(cube_1);
    for (int i=0; i < 3; i++)
    coordinates.data.insert(coordinates.data.end(), coor[i]);
    coor = get_coordinates(cube_2);
    for (int i=0; i < 3; i++)
    coordinates.data.insert(coordinates.data.end(), coor[i]);
    coor = get_coordinates(plate);
    for (int i=0; i < 3; i++)
    coordinates.data.insert(coordinates.data.end(), coor[i]);
    coor = get_coordinates(banana);
    for (int i=0; i < 3; i++)
    coordinates.data.insert(coordinates.data.end(), coor[i]);
    coor = get_coordinates(coffee);
    for (int i=0; i < 3; i++)
    coordinates.data.insert(coordinates.data.end(), coor[i]);

    cout << '\n';
    cout << '\n';
    cout << '\n';
    msg.data = solution;
    for (int k=0; k<2; k++ ){
      std::cout << "Layer" << '\n';
      for (int i=0; i<5; i++){
        for (int j=0; j<3; j++){
          cout << coordinates.data[k*5*3 + i*3 + j] << " ";
        }
        cout << '\n';
      }
    }
    publisher_alexa.publish(msg);
    coordinates_publisher.publish(coordinates);

  }
  Vec3f get_coordinates(const TransformStampedConstPtr& object){
    Vec3f coor;
    coor[0] = object->transform.translation.x;
    coor[1] = object->transform.translation.y;
    coor[0] = object->transform.translation.z;
    return coor;
  }
  Vec3f get_difference(Vec3f contacts, const TransformStampedConstPtr& object){
    Vec3f diff;
    diff[0] = contacts[0] - object->transform.translation.x;
    diff[1] = contacts[1] - object->transform.translation.y;
    diff[0] = contacts[2] - object->transform.translation.z;
    return diff;

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

  vector<Contact> getPointingObject(const TransformStampedConstPtr& object,
                                    const TransformStampedConstPtr& wand){
    shared_ptr<Sphere> object_sphere(new Sphere(0.1));
    shared_ptr<Cylinder> wand_cylinder(new Cylinder(0.1, 10));

    GJKSolver_libccd solver;
    Vec3f contact_points;
    FCL_REAL penetration_depth;
    Vec3f normal;


    Transform3f tf_object, tf_table, tf_wand;
    tf_object = setTranformation(object);
    tf_wand = setTranformation(wand);

    bool res = solver.shapeIntersect(*object_sphere, tf_object, *wand_cylinder, tf_wand,
                                     &contact_points, &penetration_depth, &normal);
    static const int num_max_contacts = std::numeric_limits<int>::max();
    static const bool enable_contact = true;
    CollisionResult result;
    CollisionRequest request(num_max_contacts, enable_contact);

    CollisionObject co0(object_sphere, tf_object);
    CollisionObject co1(wand_cylinder, tf_wand);

    collide(&co0, &co1, request, result);
    vector<Contact> contacts;
    result.getContacts(contacts);

    return contacts;
  }
//private:
  ros::NodeHandle n;
  ros::Publisher publisher_alexa;
  ros::Publisher coordinates_publisher;
  message_filters::Subscriber<TransformStamped> *cube_subscriber_1;
  message_filters::Subscriber<TransformStamped> *cube_subscriber_2;
  message_filters::Subscriber<TransformStamped> *plate_subscriber;
  message_filters::Subscriber<TransformStamped> *banana_subscriber;
  message_filters::Subscriber<TransformStamped> *coffee_subscriber;
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
