#include <limits> include <vector> include <iostream> include <cassert>
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "pid.h"

#include "my_bot.h"


using namespace Stg;
static const bool UseOtherRobotPose = false;
static const bool UseLaser = true;
static const double followdist = 3;
static const double cruisespeed = 0.4;
static const double avoidspeed = 0.05;
static const double avoidturn = 0.5;
static const double minfrontdistance = 1.0; // 0.6
static const double stopdist = 0.3;
static const int avoidduration = 10;
static const int numberOfPosHist = 10;
int maxblobx = 80;
static bool isahead = true;
typedef struct {
  ModelPosition *pos;
  ModelRanger *laser;
  ModelBlobfinder *blob;
  Stg::Pose posesAvgLastOther;
  Stg::Pose* posesOther;
  int TheposesOtherNum;
  Stg::Pose* lastDestination;
  int state; // 1 means blob detected, 0 not detected
  PID pidcruse;
  PID pidturn;
  Model* otherRobot;
  float degreeD;
  double distance;
  int avoidcount, randcount;
} robot_t;


int counter = 0;
int BlobUpdate(Model *, robot_t *robot);
int LaserUpdate(Model *mod, robot_t *robot);
int setSpeed(robot_t *robot, Stg::Pose destination);

// Stage calls robotthis when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *)
{
  // local arguments
  /*  printf( "\nWander controller initialised with:\n"
      "\tworldfile string \"%s\"\n"
      "\tcmdline string \"%s\"",
      args->worldfile.c_str(),
      args->cmdline.c_str() );
  */

  robot_t *robot = new robot_t();

  // robot->pidcruse = PID(cruisespeed, -cruisespeed, 0.1, 0.01, 0.5);
  if (isahead){
    robot->pidturn.set(avoidturn*2, -avoidturn*2, avoidturn, 0.041, 0.1);
    robot->pidcruse.set(cruisespeed*4, -cruisespeed*2, -cruisespeed  , .0100, 0.010);
  }
  // else {
  //   robot->pidturn.set(avoidturn, -avoidturn, 0.04, 0.0005, 0.000);
  //   robot->pidcruse.set(0, cruisespeed, cruisespeed*4, 1.001, 0.001);
  // }
  robot->avoidcount = 0;
  robot->posesAvgLastOther  = Stg::Pose(0.,0.,0.,0.);
  robot->posesOther  = new Stg::Pose[numberOfPosHist];
  robot->TheposesOtherNum = 0;
  robot->lastDestination  = new Stg::Pose[numberOfPosHist];
  robot->randcount = 0;
  robot->distance = 0;
  robot->state = 0;
  robot->degreeD = 0;
  robot->pos = dynamic_cast<ModelPosition *>(mod);
  if (!robot->pos) {
    PRINT_ERR("No position model given in wander controller.");
    exit(1);
  }
  World* myWorld = ((Model*)robot->pos)->GetWorld();
  robot->otherRobot = myWorld->GetModel("r1");
  // std::set< Model *> allModels =   myWorld->GetAllModels ();
  // std::cout << allModels.size() << std::endl;
  // std::set< Model *>::iterator it;
  // for (it = allModels.begin(); it != allModels.end(); ++it)
  // {
  //   std::cout << (*it)->GetId()<< std::endl;
  // }
  robot->pos->Subscribe(); // starts the position updates


  robot->laser = (ModelRanger*)mod->GetChild("ranger:1");
  robot->laser->AddCallback( Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
  robot->laser->Subscribe();


  robot->blob = dynamic_cast<ModelBlobfinder *>(robot->pos->GetChild("blobfinder:0"));//(robot->pos->GetWorld()->GetModel("blob"));
  if (!robot->blob)
    std::cout << "error" << "\n";
  robot->blob->AddCallback(Model::CB_UPDATE, model_callback_t(BlobUpdate), robot);
  robot->blob->Subscribe();
  return 0; // ok
}

int BlobUpdate(Model * mod, robot_t *robot)
{
  if (UseLaser)
    return 0;
  double speedX = 0;
  double speedY = 0;
  // no blob
  // if (counter%10 !=0)
  //   return 0;
  maxblobx = robot->blob->scan_height;
  std::cout << "maxhight: " << robot->blob->scan_width << std::endl;

  if (UseOtherRobotPose){

    double x = robot->otherRobot->GetGlobalPose().x;
    double y = robot->otherRobot->GetGlobalPose().y;
    speedX = x - robot->posesOther[0].x; 
    speedY = y - robot->posesOther[0].y;          
    
    robot->posesOther[0] = Stg::Pose(x,y,speedX,speedY);

    double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
    double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
    robot->lastDestination[0] = Stg::Pose(x+speedX*2 + speedXU*followdist, y+speedY*2 + speedYU*followdist, 0.,0.);
    ModelPosition::Waypoint wp(robot->lastDestination[0],
                                             Color("blue"));

  }
  else if (!robot->blob->GetBlobs().size() ){
    speedX = robot->posesOther[0].z;
    speedY = robot->posesOther[0].a;
    robot->lastDestination[0] = Stg::Pose(robot->lastDestination[0].x+speedX,robot->lastDestination[0].y+speedY,0,0);

  }
  else{

    double f = maxblobx / tan(robot->blob->fov/2)/2;
    
    std::cout << "Focal: " << f << std::endl;
    double range = robot->blob->GetBlobs()[0].range;
    int centerPointX = (robot->blob->GetBlobs()[0].left + robot->blob->GetBlobs()[0].right) /2;
    int centerPointY = (robot->blob->GetBlobs()[0].top + robot->blob->GetBlobs()[0].bottom) /2;
    // double x = range / f * (centerPointX - maxblobx/2);
    // double y = range; // sqrt(range*range - x*x);
    std::cout << "FOV: " << robot->blob->fov * 180 / M_PI << std::endl;
    double degree = atan2(
                          tan(robot->blob->fov/2) * (centerPointX - maxblobx/2) * 2, //(2.0 * centerPointX / maxblobx - 1)
                          maxblobx
                        );

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = K.at<float>(1, 1) = f;

    cv::Mat u(3, 1, CV_32F);
    u.at<float>(0, 0) = centerPointX - robot->blob->scan_width/2;
    u.at<float>(1, 0) = centerPointY - robot->blob->scan_height/2;
    u.at<float>(2, 0) = 1;
    u = u / cv::norm(u);

    // cv::Mat u_inv = K.inv() * u;
    // cv::Mat vertical(3, 1, CV_32F);
    // vertical.at<float>(0, 0) = vertical.at<float>(1, 0) = 0;
    // vertical.at<float>(2, 0) = 1;
    // double degree = acos(u_inv.dot(vertical) / norm(u_inv));
    
    
    // double degree = robot->blob->fov / sqrt( pow(robot->blob->scan_height, 2) + pow(robot->blob->scan_width, 2) ) * (centerPointX - maxblobx/2);

    //asin(1.0 / sqrt(3) * (2.0 * centerPointX / maxblobx - 1));
   
    // ------------------------ Exact relative ------------------------ //
    
    cv::Mat robot_pose = cv::Mat::eye(3, 3, CV_32F);
    robot_pose.at<float>(0, 0) = cos(robot->pos->GetPose().a);
    robot_pose.at<float>(0, 1) = -sin(robot->pos->GetPose().a);
    robot_pose.at<float>(1, 0) = -robot_pose.at<float>(0, 1);
    robot_pose.at<float>(1, 1) = robot_pose.at<float>(0, 0);
    robot_pose.at<float>(0, 2) = robot->pos->GetPose().x;
    robot_pose.at<float>(1, 2) = robot->pos->GetPose().y;

    cv::Mat r1_position_global(3, 1, CV_32F);
    r1_position_global.at<float>(0, 0) = robot->otherRobot->GetGlobalPose().x;
    r1_position_global.at<float>(1, 0) = robot->otherRobot->GetGlobalPose().y;
    r1_position_global.at<float>(2, 0) = 1;

    cv::Mat r1_position_local = robot_pose.inv() * r1_position_global;

    double exact_degree = M_PI - (
                                atan2 (
                                  robot->pos->GetPose().y - robot->otherRobot->GetGlobalPose().y,
                                  robot->pos->GetPose().x - robot->otherRobot->GetGlobalPose().x
                                  
                                ) + robot->pos->GetPose().a);
                          

    // ------------------------ Estimated relative ------------------------ //
    double relative_x_est = -range * cos(degree);
    double relative_y_est = range * sin(degree);

    cv::Mat relative_est(3, 1, CV_32F);
    relative_est.at<float>(0, 0) = relative_x_est;
    relative_est.at<float>(1, 0) = relative_y_est;
    relative_est.at<float>(2, 0) = 1;

    cv::Mat global_est = robot_pose * relative_est;

    double x = global_est.at<float>(0, 0);
    double y = global_est.at<float>(1, 0);
    
    // double x =  -range * cos(degree) * cos(robot->pos->GetPose().a) -
    //             range * sin(degree) * sin(robot->pos->GetPose().a) +
    //             robot->pos->GetPose().x;

    // double y =  -range * cos(degree) * sin(robot->pos->GetPose().a) +
    //             range * sin(degree) * cos(robot->pos->GetPose().a) +
    //             robot->pos->GetPose().y;



    std::cout << "Error: " 
              << "(" << degree * 180 / M_PI << " VS " << exact_degree * 180 / M_PI << "), "
              << "(" << r1_position_local.at<float>(0, 0) << ", " << r1_position_local.at<float>(1, 0) << ") VS "
              << "(" << relative_x_est << ", " << relative_y_est << "): "
              << "(" << fabs(relative_x_est - r1_position_local.at<float>(0, 0)) / 0.5 << ", " 
              << fabs(relative_y_est - r1_position_local.at<float>(1, 0)) / 0.5 << ")"
              << std::endl;

    // std::cout << "Error: " 
    //           << degree * 180 / M_PI << " "
    //           << "(" << robot->otherRobot->GetGlobalPose().x << ", " << robot->otherRobot->GetGlobalPose().y << ") VS "
    //           << "(" << x << ", " << y << ": "
    //           << fabs(x - robot->otherRobot->GetGlobalPose().x) / 0.5 << ", " 
    //           << fabs(y - robot->otherRobot->GetGlobalPose().y) / 0.5 
    //           << std::endl;
        std::cout << "counter: " << counter%4 << std::endl;
        counter++;
    switch(counter%4){
      case 0:{ robot->posesOther[0].x = x; robot->posesOther[0].y=y;
    ModelPosition::Waypoint wp(Stg::Pose(x,y,robot->otherRobot->GetGlobalPose().z,0.),
                                             Color("red"));
    ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
              return 0;
      } 
      case 1: {robot->posesOther[0].x = (robot->posesOther[0].x + x)/2; 
                    robot->posesOther[0].y = (robot->posesOther[0].y + y)/2;
      ModelPosition::Waypoint wp(Stg::Pose(x,y,robot->otherRobot->GetGlobalPose().z,0.),
                                                   Color("red"));
          ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
                    return 0;
            }
      case 2: {robot->posesOther[1].x = x; robot->posesOther[1].y=y;
      ModelPosition::Waypoint wp(Stg::Pose(x,y,robot->otherRobot->GetGlobalPose().z,0.),
                                                   Color("red"));
          ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
                    return 0;
            }
      case 3: {x = (robot->posesOther[1].x + x)/2; 
                    y = (robot->posesOther[1].y + y)/2;
                    break;}
    }

    ModelPosition::Waypoint wp(Stg::Pose(x,y,robot->otherRobot->GetGlobalPose().z,0.),
                                             Color("green"));
    ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);

    speedX = x - robot->posesOther[0].x; 
    speedY = y - robot->posesOther[0].y;          
    if (speedX == 0 && speedY == 0){
      speedX = robot->posesOther[0].z;
      speedY = robot->posesOther[0].a;
    }

    double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
    double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
    robot->lastDestination[0] = Stg::Pose(x+speedX*2 + speedXU*followdist, y+speedY*2 + speedYU*followdist, 0.,0.);
    robot->posesOther[0] = Stg::Pose(x,y,speedX,speedY);          

    ModelPosition::Waypoint wp1(robot->lastDestination[0],
                                             Color("blue"));
    ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp1);
    std::cout <<  maxblobx /2 - centerPointX << " range:" << range <<"\n" 
          // <<  robot->blob->GetBlobs()[0].left<< " right:" 
          // << robot->blob->GetBlobs()[0].right  <<  " " 
          // << robot->blob->GetBlobs()[0].range << " "  
          << "pose dest: " << robot->lastDestination[0].x << " " << robot->lastDestination[0].y << "\n"               
          << "pose secn: " << x << " " << y << " rotation:" << robot->pos->GetPose().a * 180 / M_PI  << "\n" 
          << "speed secn: " << speedX << " " << speedY << "\n" 
          << std::endl;
    }

  setSpeed(robot,robot->lastDestination[0]);

  return 0; // run again
}
 

 int getOtherRobotPose(const Stg::ModelRanger::Sensor& sensor,  robot_t *robot, Stg::Pose &otherRobot)
{
    // get the data
    const std::vector<Stg::meters_t>& scan = sensor.ranges;
  
    uint32_t sample_count = scan.size();
    if( sample_count < 1 )
        return 1;

    double laser_orientation =  robot->pos->GetPose().a - sensor.fov/2.0;
    double angle_increment = sensor.fov/(double)(sensor.sample_count-1);
    double other_x_avg = 0.0;
    double other_y_avg = 0.0;
    double num_point = 0.0;
    for (uint32_t i = 0; i < sample_count; i++) {
        // normalize the angle
        laser_orientation = atan2(sin(laser_orientation), cos(laser_orientation));
        
        double laser_x, laser_y;
        int laser_grid_x, laser_grid_y;

        laser_x =  robot->pos->GetPose().x +  scan[i] * cos(laser_orientation);
        laser_y =  robot->pos->GetPose().y +  scan[i] * sin(laser_orientation);
        
        // if (convertToGridCoords(laser_x, laser_y, laser_grid_x, laser_grid_y)) {
        //     continue;
        // }


        if ( scan[i] < (sensor.range.max - std::numeric_limits<float>::min()) ) {
          num_point++;
          other_x_avg += laser_x;
          other_y_avg += laser_y;
        }

        laser_orientation += angle_increment;
    }
    std::cout << "number point: " << num_point << std::endl;
    if (num_point){
      std::cout << "get other robot pos xy: "<< other_x_avg/num_point << " " << other_y_avg/num_point << std::endl;
      otherRobot = Stg::Pose(other_x_avg/num_point,other_y_avg/num_point,robot->otherRobot->GetGlobalPose().z,0.);
      ModelPosition::Waypoint wp(Stg::Pose(other_x_avg/num_point,other_y_avg/num_point,robot->otherRobot->GetGlobalPose().z,0.), Color("green"));
      ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
    }
    else {
     return 1;
    }
    

    return 0;
}

// inspect the ranger data and decide what to do
bool firstTime = true;
int laserCounter =0;
int LaserUpdate(Model *, robot_t *robot)
{
  if (!UseLaser)
    return 0;


  int posNum = robot->TheposesOtherNum % numberOfPosHist;
  Stg::Pose otherRobot;
  if (getOtherRobotPose (robot->laser->GetSensors()[0], robot, otherRobot)) // no position found
    return 0;
  double xTemp = otherRobot.x;
  double yTemp =  otherRobot.y;
  if (firstTime){
    robot->posesAvgLastOther = Stg::Pose(xTemp,yTemp,0,0);
    for (int i=0; i<numberOfPosHist ; i++){
       robot->posesOther[i] = Stg::Pose(xTemp,yTemp,0,0);
       robot->lastDestination[i] = Stg::Pose(xTemp,yTemp,0,0);
       std::cout << "XY temp: " << xTemp << " " << yTemp << " "<< std::endl;
    }
    firstTime =false;
  }
  robot->posesOther[posNum] = Stg::Pose(xTemp,yTemp,0,0);

  //-----------------------gitting pos history average-------------------------------
  double sumX = 0.0;
  double sumY = 0.0;
  for (int i=0; i<numberOfPosHist ; i++){
    sumX += robot->posesOther[i].x;
    sumY += robot->posesOther[i].y; 
  }

  double x = sumX/(double)numberOfPosHist;
  double y = sumY/(double)numberOfPosHist;

  double speedX = x - robot->posesAvgLastOther.x; 
  double speedY = y - robot->posesAvgLastOther.y;        
   if (speedX == 0 && speedY == 0){
    speedX = robot->posesAvgLastOther.z;
    speedY = robot->posesAvgLastOther.a;
  }
  robot->posesAvgLastOther = Stg::Pose(x,y,speedX,speedY);
  robot->posesOther[posNum] = Stg::Pose(xTemp,yTemp,0,0);
  if (speedX*speedX + speedY*speedY == 0){   //to avoid devide by 0
    return 0;
  }
  double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
  double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
  robot->lastDestination[posNum] = Stg::Pose(x+speedX*2 + speedXU*followdist, y+speedY*2 + speedYU*followdist, 0.,0.);
  //-----------------------gitting dest history average focus on last destination---------
  Stg::Pose avgDestinations= Stg::Pose(0,0,0,0);
  double devide = 1;
  for (int i=0 ; i<numberOfPosHist-1 ; i++){
    devide *= 0.5;
    avgDestinations.x += robot->lastDestination[(posNum-i+ numberOfPosHist)%numberOfPosHist].x * devide;
    avgDestinations.y += robot->lastDestination[(posNum-i + numberOfPosHist)%numberOfPosHist].y * devide;

  } 
  avgDestinations.x += robot->lastDestination[(posNum+numberOfPosHist-(numberOfPosHist-1))%numberOfPosHist].x * devide;
  avgDestinations.y += robot->lastDestination[(posNum + numberOfPosHist -(numberOfPosHist-1))%numberOfPosHist].y * devide;

  setSpeed(robot, avgDestinations);
  robot->TheposesOtherNum ++;

  return 0;
 }


int setSpeed(robot_t *robot, Stg::Pose destination){

  ModelPosition::Waypoint wp(destination ,Color("blue"));
  ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);

  double m = (destination.y - robot->pos->GetPose().y) / (destination.x - robot->pos->GetPose().x);
  double degreeD = atan(m);

  if (destination.x < robot->pos->GetPose().x && destination.y > robot->pos->GetPose().y  )
    degreeD = M_PI + degreeD;  
  else if (destination.x < robot->pos->GetPose().x && destination.y < robot->pos->GetPose().y  )
    degreeD = -M_PI + degreeD;

  if (abs(degreeD -robot->degreeD) >= 180){
    std::cout << "degreed + .: " << degreeD -robot->degreeD <<std::endl;
    degreeD = robot->degreeD;
  }
  robot->degreeD = degreeD;
  std::cout << "m: "<< m << "atan: " <<  degreeD*180 / M_PI << std::endl;
  
  double Dturn = robot->pos->GetPose().a - degreeD;

  if (Dturn >= M_PI)
    Dturn -= M_PI*2;
  else if (Dturn <= -M_PI)
     Dturn += M_PI*2;
  std::cout << "dturn: " << Dturn << std::endl;
  //set turn speed
  robot->pos->SetTurnSpeed(
    robot->pidturn.calculate( 0 ,  Dturn , 0.01 )
  );


  robot->distance = sqrt(pow(destination.x- robot->pos->GetPose().x, 2) +  pow(destination.y - robot->pos->GetPose().y, 2));
  std::cout<< "distance: " << robot->distance << std::endl;
  double Xspeed = robot->pidcruse.calculate(0 ,robot->distance, 0.01 );
  std::cout << "Xspeed: " << Xspeed << std::endl;
  robot->pos->SetXSpeed(Xspeed);

  return 0; // run again
}