#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fcl/collision.h>
#include <fcl/narrowphase/narrowphase.h>

using namespace std;
using namespace fcl;
using namespace std_msgs;
using namespace geometry_msgs;
using namespace message_filters;

const int NUMBER_OF_OBJECTS = 5;
const int NUMBER_OF_LAYERS = 2;
const int NUMBER_OF_DIM = 3;
const string NAME_OF_OBJECTS[] = {"Cube1", "Cube2", "Plate", "Banana", "Coffee"};

typedef message_filters::sync_policies::ApproximateTime<TransformStamped,
                                                        TransformStamped,
                                                        TransformStamped,
                                                        TransformStamped,
                                                        TransformStamped,
                                                        TransformStamped,
                                                        TransformStamped>
                                                        MySyncPolicy;
typedef message_filters::Subscriber<TransformStamped> TransformStampedType;

struct Message{
   Float64MultiArray coordinates;
   String alexa;
};

class SubscribeAndPublish{

public:
  SubscribeAndPublish(){
    publisher_alexa = n.advertise<std_msgs::String>("/alexa_out", 1);
    coordinates_publisher = n.advertise<std_msgs::Float64MultiArray> ("/coordinates", 5);
    cube_subscriber_1 = new TransformStampedType(n, "vicon/Cube1/Cube1", 5);
    cube_subscriber_2 = new TransformStampedType(n, "vicon/Cube2/Cube2", 5);
    plate_subscriber = new TransformStampedType(n, "vicon/Plate/Plate", 5);
    banana_subscriber = new TransformStampedType(n, "vicon/Banana/Banana", 5);
    coffee_subscriber = new TransformStampedType(n, "vicon/Coffee/Coffee", 5);
    table_subscriber = new TransformStampedType(n, "vicon/Table/Table", 5);
    wand_subscriber = new TransformStampedType(n, "vicon/Wand/Wand", 5);

    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(5),
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

    vector<TransformStampedConstPtr> objects;
    objects.push_back(cube_1);
    objects.push_back(cube_2);
    objects.push_back(plate);
    objects.push_back(banana);
    objects.push_back(coffee);
    objects.push_back(wand);

    Message m = prepare_publish_message(objects);
    publisher_alexa.publish(m.alexa);
    coordinates_publisher.publish(m.coordinates);
  }

  Message prepare_publish_message(vector<TransformStampedConstPtr>& objects){
    Message m;
    String alexa_message;
    string solution;
    TransformStampedConstPtr wand = objects.back();
    objects.pop_back();
    vector<vector<Contact>> contacts;
    for (const TransformStampedConstPtr& object:objects){
      contacts.push_back(getPointingObject(object, wand));
    }
    Float64MultiArray coordinates = initialize_coordinates();
    for (int i=0; i < NUMBER_OF_OBJECTS; i++){
      if (!contacts[i].empty()){
        solution = output_info(contacts[i], i);
        for(const Contact &contact : contacts[i]){
          Vec3f diff = get_difference(contact.pos, objects[i]);
          for (int j=0; j < NUMBER_OF_DIM; j++)
            coordinates.data.insert(coordinates.data.end(), diff[j]);
          }
      }
      else{
        for (int j=0; j < NUMBER_OF_DIM; j++)
          coordinates.data.insert(coordinates.data.end(), 10);
        }
      }
    alexa_message.data = solution;
    for (const TransformStampedConstPtr object:objects){
      Vec3f coor = get_coordinates(object);
      for (int i=0; i < NUMBER_OF_DIM; i++)
      coordinates.data.insert(coordinates.data.end(), coor[i]);
    }
    m.alexa = alexa_message;
    m.coordinates = coordinates;
    return m;
  }

  string output_info(const vector<Contact>& contacts, const int& i){
    string solution = "The wand is pointing to " + NAME_OF_OBJECTS[i];
    cout << solution << endl;
    cout << contacts.size() <<  " contacts found" << endl;
    for(const Contact &contact : contacts){
      cout << "position: " << contact.pos << endl;
      cout << endl;
    }
    cout << endl;
    cout << endl;
    return solution;
  }

  Float64MultiArray initialize_coordinates(){
    Float64MultiArray coordinates;
    vector<int> dimensions = {NUMBER_OF_LAYERS, NUMBER_OF_OBJECTS, NUMBER_OF_DIM};
    int size = 1;
    for (int i=0; i<dimensions.size(); i++){
      size *= dimensions[i];
    }
    for (int i=0; i<dimensions.size(); i++){
      coordinates.layout.dim.push_back(std_msgs::MultiArrayDimension());
      coordinates.layout.dim[i].size = dimensions[i];
      coordinates.layout.dim[i].stride = size;
      coordinates.layout.dim[i].label = i;
      size /= dimensions[i];
    }
    coordinates.data.clear();
    return coordinates;
  }

  Vec3f get_coordinates(const TransformStampedConstPtr& object){
    Vec3f coor;
    coor[0] = object->transform.translation.x;
    coor[1] = object->transform.translation.y;
    coor[0] = object->transform.translation.z;
    return coor;
  }

  Vec3f get_difference(const Vec3f& contacts, const TransformStampedConstPtr& object){
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

  ros::NodeHandle n;
  ros::Publisher publisher_alexa;
  ros::Publisher coordinates_publisher;
  Subscriber<TransformStamped> *cube_subscriber_1;
  Subscriber<TransformStamped> *cube_subscriber_2;
  Subscriber<TransformStamped> *plate_subscriber;
  Subscriber<TransformStamped> *banana_subscriber;
  Subscriber<TransformStamped> *coffee_subscriber;
  Subscriber<TransformStamped> *table_subscriber;
  Subscriber<TransformStamped> *wand_subscriber;
  Synchronizer<MySyncPolicy>* sync;
};

int main(int argc, char **argv){

  ros::init(argc, argv, "subscribe_and_publish");
  SubscribeAndPublish SAPObject;
  ros::Rate r(1);
  while (true){
    ros::spinOnce();
    r.sleep();
   }
  return 0;
}
