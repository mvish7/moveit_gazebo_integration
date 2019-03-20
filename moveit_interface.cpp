
//ROS
#include "ros/ros.h"
//moveit
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
//for inclusion of mesh as geometric shape
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//gazebo dependencies
#include "gazebo_msgs/GetModelState.h"



void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}


void pick(moveit::planning_interface::MoveGroupInterface& move_group, float model_pose[])
{   

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

//setting pre grasp pose i.e ask robot arm to go to this position and wait before approaching the object
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;

    orientation.setRPY(-M_PI / 4, -M_PI / 2, -M_PI / 4);

    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x =  model_pose[0];
    grasps[0].grasp_pose.pose.position.y =  model_pose[1]-0.10;
    grasps[0].grasp_pose.pose.position.z =  model_pose[2]+0.06;  //notice the z offset i found out that gripper tries to grasp object at its base. 
                                                          //This is happening because of gazebo coordinate system gives pose of the object considering its origin at the base of the object 

 //setting pre grasp approach i.e asking robot to approach the object before trying to grasp it   

     grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive y axis */
     grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
     grasps[0].pre_grasp_approach.min_distance = 0.095;
     grasps[0].pre_grasp_approach.desired_distance = 0.110;     

 // Setting post-grasp retreat 

     grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
     grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
     grasps[0].post_grasp_retreat.min_distance = 0.01;
     grasps[0].post_grasp_retreat.desired_distance = 0.025;
   

//setting posture of end effector before grasp

     openGripper(grasps[0].pre_grasp_posture);

//setting posture of eef during grasp

     closedGripper(grasps[0].grasp_posture);

// Set support surface as table1.
   
     move_group.setSupportSurfaceName("shelf");

// Call pick to pick up the object using the grasps given
     
     move_group.pick("kaffee", grasps); 
} 



void place(moveit::planning_interface::MoveGroupInterface& group)
{
 
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  //orientation.setRPY(0, 0, M_PI / 2);
  orientation.setRPY(M_PI , 0 , 0 );

  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  place_location[0].place_pose.pose.position.x = 0.298225;
  place_location[0].place_pose.pose.position.y = -0.539508; // do we need some y offset??
  place_location[0].place_pose.pose.position.z = 0.236279+0.08; //offset to keep object on table  

  // Setting pre-place approach
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].pre_place_approach.direction.vector.y = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive y axis */
  place_location[0].post_place_retreat.direction.vector.y = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

   /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table");
  // Call place to place the object using the place locations given.
  group.place("kaffee", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);
    
    shapes::Mesh* m1 = shapes::createMeshFromResource("package://robot_description/meshes/RVIZ/shelf_stl.stl");
    ROS_INFO("shelf mesh loaded");

    sleep(2.0);

    shape_msgs::Mesh shelf_mesh;
    shapes::ShapeMsg shelf_mesh_msg;
    shapes::constructMsgFromShape(m1, shelf_mesh_msg);
    shelf_mesh = boost::get<shape_msgs::Mesh>(shelf_mesh_msg);
    collision_objects[0].id = "shelf";
    collision_objects[0].header.frame_id = "world";
    sleep(2.0);

    collision_objects[0].meshes.resize(1);
    collision_objects[0].mesh_poses.resize(1);
    collision_objects[0].meshes[0] = shelf_mesh;
    collision_objects[0].mesh_poses[0].position.x = 0.283853;
    collision_objects[0].mesh_poses[0].position.y = 0.730556;
    collision_objects[0].mesh_poses[0].position.z = -0.003741;
    collision_objects[0].mesh_poses[0].orientation.x = 0.999850;
    collision_objects[0].mesh_poses[0].orientation.y= 0.006747;
    collision_objects[0].mesh_poses[0].orientation.z= 0.000108;
    collision_objects[0].mesh_poses[0].orientation.w= 0.015920;

    collision_objects[0].meshes.push_back(shelf_mesh);
    collision_objects[0].mesh_poses.push_back(collision_objects[0].mesh_poses[0]);
    collision_objects[0].operation = collision_objects[0].ADD;

    // adding kaffee mesh

    shapes::Mesh* m2 = shapes::createMeshFromResource("package://robot_description/meshes/RVIZ/kaffee.stl");
    ROS_INFO("kaffee mesh loaded");

    sleep(2.0);

    shape_msgs::Mesh kaffee_mesh;
    shapes::ShapeMsg kaffee_mesh_msg;
    shapes::constructMsgFromShape(m2, kaffee_mesh_msg);
    kaffee_mesh = boost::get<shape_msgs::Mesh>(kaffee_mesh_msg);
    collision_objects[1].id = "kaffee";
    collision_objects[1].header.frame_id = "world";
    sleep(2.0);

    collision_objects[1].meshes.resize(1);
    collision_objects[1].mesh_poses.resize(1);
    collision_objects[1].meshes[0] = kaffee_mesh;
    collision_objects[1].mesh_poses[0].position.x = 0.325435;
    collision_objects[1].mesh_poses[0].position.y = 0.496533;
    collision_objects[1].mesh_poses[0].position.z = 0.422545;
    collision_objects[1].mesh_poses[0].orientation.x = -0.026790;
    collision_objects[1].mesh_poses[0].orientation.y= -0.001546;
    collision_objects[1].mesh_poses[0].orientation.z= 0.015858;
    collision_objects[1].mesh_poses[0].orientation.w= 0.999514;
    

    collision_objects[1].meshes.push_back(kaffee_mesh);
    collision_objects[1].mesh_poses.push_back(collision_objects[1].mesh_poses[0]);
    collision_objects[1].operation = collision_objects[1].ADD;   
    


    shapes::Mesh* m3 = shapes::createMeshFromResource("package://robot_description/meshes/RVIZ/table.stl");
    ROS_INFO("table mesh loaded");

    sleep(2.0);

    shape_msgs::Mesh table_mesh;
    shapes::ShapeMsg table_mesh_msg;
    shapes::constructMsgFromShape(m3, table_mesh_msg);
    table_mesh = boost::get<shape_msgs::Mesh>(table_mesh_msg);
    collision_objects[2].id = "table";
    collision_objects[2].header.frame_id = "world";
    sleep(2.0);

    collision_objects[2].meshes.resize(1);
    collision_objects[2].mesh_poses.resize(1);
    collision_objects[2].meshes[0] = table_mesh;
    collision_objects[2].mesh_poses[0].position.x = 0.311825;
    collision_objects[2].mesh_poses[0].position.y = -0.652401;
    collision_objects[2].mesh_poses[0].position.z = 0.000757;
    collision_objects[2].mesh_poses[0].orientation.x = 0.013110;
    collision_objects[2].mesh_poses[0].orientation.y= -0.003755;
    collision_objects[2].mesh_poses[0].orientation.z= -0.000415;
    collision_objects[2].mesh_poses[0].orientation.w= 0.999907;
    

    collision_objects[2].meshes.push_back(table_mesh);
    collision_objects[2].mesh_poses.push_back(collision_objects[2].mesh_poses[0]);
    collision_objects[2].operation = collision_objects[2].ADD;    


    std::vector<moveit_msgs::CollisionObject> collision_vector;
    //collision_vector.resize(2);
    collision_vector.push_back(collision_objects[0]);  
    ROS_INFO("shelf added into the world");  

    collision_vector.push_back(collision_objects[1]);  
    ROS_INFO("kaffee added into the world"); 

    collision_vector.push_back(collision_objects[2]);  
    ROS_INFO("table added into the world"); 
    
    planning_scene_interface.applyCollisionObjects(collision_vector);
    sleep(5.0);

}

float * fetch_model_poses(ros::NodeHandle& nh, std::string & str)
{ 
  static float pose [7];
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  
  gazebo_msgs::GetModelState getmodelstate;
  getmodelstate.request.model_name= str;

  if (client.call(getmodelstate))
  {  
      pose[0]= getmodelstate.response.pose.position.x;
      pose[1]= getmodelstate.response.pose.position.y;
      pose[2]= getmodelstate.response.pose.position.z;
      pose[3]= getmodelstate.response.pose.orientation.x;
      pose[4]= getmodelstate.response.pose.orientation.y;
      pose[5]= getmodelstate.response.pose.orientation.z;
      pose[6]= getmodelstate.response.pose.orientation.w;
 
      ROS_INFO("fetched model pose");    

      return pose;   
  }


    else
  {
      ROS_INFO ("pose fetch failed");

   //   return float 1.1;

  }

   

}



int main(int argc, char** argv)
{ 

  float *model_pose;
  //std::cout << "Please enter model name ";
  
  std::string model_name;
  std::string param;
 // std::cin >> model_name;
  //std::cout << "Your name is: " << model_name << std::endl;
  
  //ROS_INFO("requested model is %s", model_name);

  ros::init(argc, argv, "moveit_interface");
  ros::NodeHandle nh("~");
  nh.getParam("param", param);
  ROS_INFO("Got parameter : %s", param.c_str());
  model_name=param.c_str();
  
  ros::AsyncSpinner spinner(1);
  spinner.start();


  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);

  model_pose=fetch_model_poses(nh, model_name);


  addCollisionObjects(planning_scene_interface);


  // Wait a bit for ROS things to initialize
  ros::WallDuration(2.0).sleep();

  pick(group, model_pose);

  place(group);

  ros::waitForShutdown();
  return 0;

}
