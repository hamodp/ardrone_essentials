/********************************************************************************************
* ar_cam class
* 
* This class is designed to work with the ar drone parrot 2.0 to change between the front 
* camera and bottom camera with minimal delay and provide more accurate topics for the top
* bottom cam as well as a bool value to determine if it is on the bottom cam or not
*
* Authors: Patrick Hamod
* Last Edited: 6 April 2016
* 
**/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"

#include <cstdlib>

#include <ctime>
#include <sstream>

/**
 * This program finds the frame rate of a camera
 * publishing to the topic sensor_msgs/Image
 */
class ar_cam
{
  public:
  
  ar_cam(ros::NodeHandle& nh)
  {
    prevTimen = 0;
    prevTime= 0;
    fps = 0;
    maxTime = 0;
    isDucReady = false;
    isBottomCam = false;
    justSwitched = false;
    startSwitching=false;
    framelimit = 0;
    
 
  //change this to another sensor_msgs/Image topic
    imagesub = nh.subscribe("/ardrone/image_raw", 100000, &ar_cam::imageCallback, this);
    startChange=nh.subscribe("/ardrone/cameraswap", 100, &ar_cam::commandCallback, this);
    cameraInUse= nh.advertise<std_msgs::Bool>("ardrone/cameraInUse",1);
    bottomimage = nh.advertise<sensor_msgs::Image>("ardrone/bottom_cam", 1);
    frontimage = nh.advertise<sensor_msgs::Image>("ardrone/front_cam", 1);
    client = nh.serviceClient<std_srvs::Empty>("ardrone/togglecam");
    
    
    std_srvs::Empty srv;

  }


  /********************************************************************************
  * function for determining if it should change or or not expects a string message
  * with data = to start
  */
  void commandCallback(const std_msgs::StringConstPtr str)
  {
    
    if(str->data.size()==5 && str->data.substr(0,5) == "start")
    {
      startSwitching=true;
    }
  }

  void changeCam()
  {
    if(startSwitching)
    {
      justSwitched = true;
      client.call(srv);
      isBottomCam = !isBottomCam;
      usleep(100);
    }
  }

/**********************************************************************
* Function to find the change of time before the next arrival of a frame
* in nanoseconds
*
*@Parameter: const sensor_msgs::Image::ConstPtr& msg is the sensor_msgs/Image
*            that the program is subscribed to
*/
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
    //print the delay if the camera is switching
    if(startSwitching)
    {
      double deltaTime = 0;
      double deltaTimen =0;
      double nextTimen = msg->header.stamp.nsec;
      fps++;
  
      double nextTime = msg->header.stamp.sec;
      deltaTimen = nextTimen-prevTimen;
      deltaTime = (nextTime - prevTime)*1000000;
  
      deltaTime += deltaTimen;
      prevTime = nextTime;
      prevTimen = nextTimen;

  
      //if(deltaTime >= maxTime && isDucReady)
      //{
        maxTime = deltaTime;
        std::stringstream ss;
        ss << "change of time: " << deltaTime<< std::endl;
        ROS_INFO(ss.str().c_str());
      //}
      isDucReady= true;
    
      //if the camera just changed give a few frames to minimize confused frames
      if(justSwitched)
      {
      
        framelimit = (framelimit+1)%6;
        usleep(100);
        if(framelimit == 0)
        {
          justSwitched=false;
        
        }
      }
      //if it is bottom cam publish the message to bottom cam
      else if(isBottomCam)
      {
        std_msgs::Bool camera;
        camera.data = isBottomCam;
        bottomimage.publish(msg);
        cameraInUse.publish(camera);
      
      }
      //otherwise publish to the top cam
      else if(!isBottomCam)
      {
        std_msgs::Bool camera;
        camera.data = isBottomCam;
        frontimage.publish(msg);
        cameraInUse.publish(camera);
      }
    }
  }

  private:

    double prevTimen;
    double prevTime;
    int fps;
    double maxTime;
    bool isDucReady;
    ros::Subscriber imagesub;
    ros::Publisher bottomimage;
    ros::Publisher frontimage;
    ros::Publisher cameraInUse;
    ros::ServiceClient client;
    ros::Subscriber startChange;
    std_srvs::Empty srv;
    bool isBottomCam;
    bool justSwitched;
    int framelimit;
    bool startSwitching;
          
};

int main(int argc, char **argv)
{
  //create the node ros functions can be used
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  //spin once starts the timers
  ar_cam sharknado(n);
  ros::spinOnce();

  //number of test cases for average frames per second calculations
  int runs = 10;
  int sum = 0;
  ros::Rate rate(10);
  //for loop runs to get an idea about how many frames are published per second on rostopic
 /* for(int i = 0; i<runs ; i++)
  {
    fps =0;
    ros::Time startTime = ros::Time::now();
    ros::Duration duration = ros::Duration(1);//set the timer to find the frame rate per second

    while(startTime+duration >= ros::Time::now())
    {
      ros::spinOnce();
    }
    sum+= fps;
    std::cout<<"Camera runs: " << fps <<" frames per second."<< std::endl;
  }*/ 
  
  std::cout<<"Camera runs: " << sum/runs <<" frames per second on average."<< std::endl;
  int switchCam = 0;
  while(ros::ok())
  {
    if(switchCam ==0)
    {
      sharknado.changeCam();
      //std::stringstream ss;
      //ss << "changed the camera: " << switchCam << std::endl;
      //ROS_INFO(ss.str().c_str());
    }
    std::stringstream ss;
    ss << "switchCam: " << switchCam << std::endl;
    //std::cout << ss.str();
    //change the camera after
    switchCam= (switchCam+1)%10;
    ros::spinOnce();
    rate.sleep();
  }
    
  return 0;
}
