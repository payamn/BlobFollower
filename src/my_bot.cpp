#include <limits> include <vector> include <iostream> include <cassert>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "pid.h"

#include "my_bot.h"
#include "kalman.hpp"
#include "utils.hpp"
#include "config.h"
#define NUM_STATES 9

//propagate uncertenty in robot pose to the person pose
using namespace Stg;

int maxblobx = 80;
static bool isahead = true;
typedef struct {
  Stg::Pose robotPose;
  ModelPosition *pos;
  ModelRanger *laser;
  ModelBlobfinder *blobF;
  ModelBlobfinder *blobB;
  Stg::Pose prevPoseMine;
  Stg::Pose relative_pose_other;

  // for visualization purpose, we save the absolute pose when the origin was fixed
  Stg::Pose robotPose_w;
  KalmanFilter *kalmanFilter;
  Stg::Pose posesAvgLastOther;
  Stg::Pose* posesOther;
  
  Stg::Pose poseOtherPrev;
  bool isOtherPoseValid;

  int TheposesOtherNum;
  Stg::Pose* lastDestination;
  int state; // 1 means blob detected, 0 not detected
  PID pidcruse;
  PID pidturn;
  Model* otherRobot;
  float degreeD;
  double distance;
  int avoidcount, randcount;

  int referenceLastUpdated;
} robot_t;


int counter = 0;
int MyBlobUpdate (robot_t*robot, bool isFront);
int BlobUpdateBack (Model *, robot_t *robot);
int BlobUpdateFront (Model *, robot_t *robot);
int LaserUpdate(Model *mod, robot_t *robot);
int PoseUpdate(Model *, robot_t *robot);
int setSpeed(robot_t *robot, Stg::Pose destination);
int getOtherRobotPoseBlob (robot_t *robot, Stg::Pose &otherRobot, ModelBlobfinder *blob, bool isFront);
int getOtherRobotPoseLaser(const Stg::ModelRanger::Sensor& sensor,  robot_t *robot, Stg::Pose &otherRobot);
int getDestinationBasedOnOtherObjectPoses(robot_t *robot, Stg::Pose otherRobot, Stg::Pose& avgDestinations);
int createWaypoint(robot_t *robot, Stg::Pose, char* color);



double getBlobBearing(robot_t *robot, double blobCoord, ModelBlobfinder *blob)
{
  return ( blobCoord * blob->fov / (blob->scan_width-1) - blob->fov/2.0);

}


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
  srand((unsigned)time(0));
  std::cout << "start init" << std::endl;
  cv::Mat Q = cv::Mat::zeros(NUM_STATES, NUM_STATES, CV_32F);
  Q.at<float>(0, 0) = Q0;
  Q.at<float>(1, 1) = Q1;
  Q.at<float>(2, 2) = Q2;
  Q.at<float>(3, 3) = Q3;
  Q.at<float>(4, 4) = Q4;
  Q.at<float>(5, 5) = Q5;
  Q.at<float>(6, 6) = Q6;
  Q.at<float>(7, 7) = Q7;
  Q.at<float>(8, 8) = Q8;

  cv::Mat R = cv::Mat::zeros(4, 4, CV_32F);
  R.at<float>(0, 0) = R0;
  R.at<float>(1, 1) = R1;
  R.at<float>(2, 2) = R2;
  R.at<float>(3, 3) = R3;

  cv::Mat P = cv::Mat::eye(NUM_STATES, NUM_STATES, CV_32F);
  P.at<float>(0, 0) = P0;
  P.at<float>(1, 1) = P1;
  P.at<float>(2, 2) = P2;
  P.at<float>(3, 3) = P3;
  P.at<float>(4, 4) = P4;
  P.at<float>(5, 5) = P5;
  P.at<float>(6, 6) = P6;
  P.at<float>(7, 7) = P7;
  P.at<float>(8, 8) = P8;

  
  
  robot_t *robot = new robot_t();
  robot->isOtherPoseValid = false;

  // robot->pidcruse = PID(cruisespeed, -cruisespeed, 0.1, 0.01, 0.5);
  if (isahead){
    robot->pidturn.set(avoidturn, -avoidturn, avoidturn, 0.0100, 0.01);
    robot->pidcruse.set(cruisespeed*3, -cruisespeed, -cruisespeed  , .0100, 0.001);
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
  robot->referenceLastUpdated = 0;
  std::cout << "before kalman init" << std::endl;
  robot->kalmanFilter = new KalmanFilter(0.1, Q, R, P);
  std::cout << "after kalamn init" << std::endl;
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
  robot->pos->AddCallback( Model::CB_UPDATE, model_callback_t(PoseUpdate), robot);
  robot->pos->Subscribe(); // starts the position updates

  robot->robotPose = Stg::Pose(0, 0, 0, 0);
  robot->prevPoseMine = Stg::Pose(0, 0, 0, 0);
  robot->robotPose_w = robot->pos->GetPose();

  robot->laser = (ModelRanger*)mod->GetChild("ranger:1");
  robot->laser->AddCallback( Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
  robot->laser->Subscribe();

  robot->blobB = dynamic_cast<ModelBlobfinder *>(robot->pos->GetChild("blobfinder:0"));//(robot->pos->GetWorld()->GetModel("blob"));
  if (!robot->blobB)
    std::cout << "error" << "\n";

  robot->blobF = dynamic_cast<ModelBlobfinder *>(robot->pos->GetChild("blobfinder:1"));//(robot->pos->GetWorld()->GetModel("blob"));
  if (!robot->blobF)
    std::cout << "error" << "\n";
  robot->blobB->AddCallback(Model::CB_UPDATE, model_callback_t(BlobUpdateBack), robot); // boolean parameter: is front blob detector
  robot->blobB->Subscribe();

  robot->blobF->AddCallback(Model::CB_UPDATE, model_callback_t(BlobUpdateFront), robot); // boolean parameter: is front blob detector
  robot->blobF->Subscribe();
  return 0; // ok
}

int calPosBasedSpeed(Stg::Pose speed, Stg::Pose &robot_pose)
{   
  robot_pose = Stg::Pose(robot_pose.x+speed.x, robot_pose.y+speed.y, 0, 0);
  return 0;
}

int myBlobUpdate ( robot_t *robot, bool isFront)
{
  ModelBlobfinder *blob;

  if (isFront)
    blob = robot->blobF;
  else
    blob = robot->blobB;
  double speedX = 0;
  double speedY = 0;
  // no blob
  // if (counter%10 !=0)
  //   return 0;
  maxblobx = blob->scan_width;
  // std::cout << "maxhight: " << blob->scan_width << std::endl;

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
    setSpeed(robot,robot->lastDestination[0]);

  }
  else if (!blob->GetBlobs().size()){
    return 0;

    //---------------- prediction to do:
    // if (firstTime)
    //   return 0;
    // Stg::Pose speed = Stg::Pose(robot->posesAvgLastOther.z, robot->posesAvgLastOther.a,0,0);
    // // speedX = robot->posesAvgLastOther.z;
    // // speedY = robot->posesAvgLastOther.a;
    // Stg::Pose otherRobot = robot->posesAvgLastOther;
    // calPosBasedSpeed(speed ,otherRobot);
    // Stg::Pose avgDestinations;
    // if (getDestinationBasedOnOtherObjectPoses(robot, otherRobot, avgDestinations))
    //   return 0;
    // ModelPosition::Waypoint wp(otherRobot, Color("red"));
    // ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
    // std::cout << "a other robot pos: " << otherRobot.x << " "<< otherRobot.y << std::endl;

    
    // setSpeed(robot,avgDestinations);

  }
  else{


    Stg::Pose otherRobot;
    if (getOtherRobotPoseBlob (robot, otherRobot, blob, isFront))
    {
      return 0;
    }
    // double x = otherRobot.x;
    // double y = otherRobot.y;
    // double x =  -range * cos(degree) * cos(robot->pos->GetPose().a) -
    //             range * sin(degree) * sin(robot->pos->GetPose().a) +
    //             robot->robotPose.x;

    // double y =  -range * cos(degree) * sin(robot->pos->GetPose().a) +
    //             range * sin(degree) * cos(robot->pos->GetPose().a) +
    //             robot->robotPose.y;




    // std::cout << "Error: " 
    //           << degree * 180 / M_PI << " "
    //           << "(" << robot->otherRobot->GetGlobalPose().x << ", " << robot->otherRobot->GetGlobalPose().y << ") VS "
    //           << "(" << x << ", " << y << ": "
    //           << fabs(x - robot->otherRobot->GetGlobalPose().x) / 0.5 << ", " 
    //           << fabs(y - robot->otherRobot->GetGlobalPose().y) / 0.5 
    //           << std::endl;
    // Stg::Pose avgDestinations;
    // if (getDestinationBasedOnOtherObjectPoses(robot, otherRobot, avgDestinations))
    // {
    //   return 0;
    // }

    // ModelPosition::Waypoint wp(Stg::Pose(x,y,robot->otherRobot->GetGlobalPose().z,0.),
    //                                          Color("green"));
    // ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);

    // speedX = x - robot->posesOther[0].x; 
    // speedY = y - robot->posesOther[0].y;          
    // if (speedX == 0 && speedY == 0){
    //   speedX = robot->posesOther[0].z;
    //   speedY = robot->posesOther[0].a;
    // }

    // double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
    // double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
    // robot->lastDestination[0] = Stg::Pose(x+speedX*2 + speedXU*followdist, y+speedY*2 + speedYU*followdist, 0.,0.);
    // robot->posesOther[0] = Stg::Pose(x,y,speedX,speedY);          

    // ModelPosition::Waypoint wp1(robot->lastDestination[0],
    //                                          Color("blue"));
    // ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp1);
    // std::cout << "pose dest: " << robot->lastDestination[0].x << " " << robot->lastDestination[0].y << "\n"               
    //       << "pose secn: " << x << " " << y << " rotation:" << robot->pos->GetPose().a * 180 / M_PI  << "\n" 
    //       << "speed secn: " << speedX << " " << speedY << "\n" 
    //       << std::endl;
    // }
    // setSpeed(robot,avgDestinations);
  }
}

int BlobUpdateFront(Model * mod, robot_t *robot)
{
  if (UseLaser)
    return 0;

  myBlobUpdate (robot, true);

  return 0;
}

int BlobUpdateBack(Model * mod, robot_t *robot)
{
  if (UseLaser)
    return 0;
  myBlobUpdate (robot, false);

  return 0; // run again
}

Stg::meters_t getRangeFromLaser(robot_t *robot, double startAngle, double endAngle)
{
  // get the data
  std::vector<Stg::meters_t> scan = robot->laser->GetSensors()[0].ranges;
  // make the orientation of laser scan counter-clockwise (+ve) to the robot
  std::reverse(scan.begin(), scan.end());

  double angleIncrement = robot->laser->GetSensors()[0].fov/(double)(robot->laser->GetSensors()[0].sample_count-1);
  size_t startIdx = round(startAngle/angleIncrement);
  size_t endIdx = round(endAngle/angleIncrement);

  // debug
  // std::cout << "Non saturated laser" << std::endl;
  // for (size_t i=0; i<scan.size(); i++)
  // {
  //   if (scan[i]<8)
  //   {
  //     std::cout << i << " ";
  //   }
  // }
  // std::cout << std::endl;

  // std::cout << "Start: " << startIdx << " End: " << endIdx << std::endl;
  
  std::vector<Stg::meters_t> validScan;
  for (size_t i=startIdx; i!=endIdx+1; i++)
  {
    i = i % robot->laser->GetSensors()[0].sample_count;
    if (scan[i] < robot->laser->GetSensors()[0].range.max)
    {
      // std::cout << scan[i] << std::endl;
      validScan.push_back(scan[i]);
    }
  }
  
  if (validScan.size() == 0)
  {
    std::cout << "No valid scan" << std::endl;
    return 0;
  }

  // median
  size_t medianIdx = validScan.size()/2;
  std::nth_element(validScan.begin(), validScan.begin()+medianIdx, validScan.end());
  return validScan[medianIdx];
  // min
  // return *(std::min_element(validScan.begin(), validScan.end()));

}


int getOtherRobotPoseBlob (robot_t *robot, Stg::Pose &otherRobot, ModelBlobfinder *blob, bool isFront)
{
    cv::Mat robot_pose = pose2TransformationMatrix(robot->prevPoseMine);

    double centerPointX = (blob->GetBlobs()[0].left + blob->GetBlobs()[0].right) / 2.0;
    double centerPointY = (blob->GetBlobs()[0].top + blob->GetBlobs()[0].bottom) / 2.0;
    // double x = range / f * (centerPointX - maxblobx/2);
    // double y = range; // sqrt(range*range - x*x);
    // std::cout << "FOV: " << blob->fov * 180 / M_PI << std::endl;
    double degree = getBlobBearing(robot, centerPointX, blob);

    // double range = blob->GetBlobs()[0].range;
    double blobStartAngle = getBlobBearing(robot, blob->GetBlobs()[0].left, blob);
    double blobEndAngle = getBlobBearing(robot, blob->GetBlobs()[0].right, blob);
    // the start of laser is at the back of robot
    double laserStartAngle = isFront ? M_PI + blobStartAngle : blobStartAngle ;
    double laserEndAngle = isFront ? M_PI + blobEndAngle: blobEndAngle;
    
    laserStartAngle = atan2(sin(laserStartAngle), cos(laserStartAngle));
    laserEndAngle = atan2(sin(laserEndAngle), cos(laserEndAngle));
    laserStartAngle = laserStartAngle>=0 ? laserStartAngle : laserStartAngle + 2*M_PI;
    laserEndAngle = laserEndAngle>=0 ? laserEndAngle : laserEndAngle + 2*M_PI;

    // debug
    // std::cout << " Blob Start: " << blobStartAngle * 180 / M_PI 
    //           << " Blob End: " << blobEndAngle * 180 / M_PI 
    //           << std::endl
    //           << " Laser Start: " << laserStartAngle * 180 / M_PI 
    //           << " Laser End: " << laserEndAngle * 180 / M_PI 
    //           << std::endl;

    double range = getRangeFromLaser(robot, laserStartAngle, laserEndAngle);
    if (range == 0)
    {
      return 1;
    }

    // std::cout << "Estimated range: " << range
    //           << " Actual Range: " << blob->GetBlobs()[0].range
    //           << std::endl;

    
    // ------------------------ Exact relative ------------------------ //
    cv::Mat r1_position_global = pose2HomgeneousVector( robot->otherRobot->GetGlobalPose() );
    cv::Mat r1_position_local = robot_pose.inv() * r1_position_global;

    double exact_degree = M_PI - (
                                atan2 (
                                  robot->robotPose.y - robot->otherRobot->GetGlobalPose().y,
                                  robot->robotPose.x - robot->otherRobot->GetGlobalPose().x
                                  
                                ) + robot->pos->GetPose().a);
                          

    // ------------------------ Estimated relative ------------------------ //
    // use exact range
    double exact_range = sqrt(
      pow(robot->robotPose.y - robot->otherRobot->GetGlobalPose().y, 2) +
      pow(robot->robotPose.x - robot->otherRobot->GetGlobalPose().x, 2)
    );
    double relative_x_est = -range * cos(degree);
    double relative_y_est = range * sin(degree);
    
    if (isFront)
    {    
      relative_x_est *=-1;
      relative_y_est *= -1;
    }

    cv::Mat relative_est(3, 1, CV_32F);
    relative_est.at<float>(0, 0) = relative_x_est;
    relative_est.at<float>(1, 0) = relative_y_est;
    relative_est.at<float>(2, 0) = 1;

    robot->relative_pose_other = Stg::Pose(relative_x_est,relative_y_est,0,0);
    // std::cout << "Exact: " << r1_position_local.t() << ", Estimated: " << relative_est.t() << std::endl;
    // std::cout << "Exact range: " << exact_range << ", Estimated: " << range << std::endl;

    cv::Mat global_est = robot_pose * relative_est;
    
    otherRobot = Stg::Pose(global_est.at<float>(0, 0), global_est.at<float>(1, 0),0,0);
    robot->poseOtherPrev = otherRobot;
    robot->isOtherPoseValid = true;

    // for visualization purpose, plot other robot estimate wrt r0 (using its absolute position/orientation)
    // transformation from r0 to world frame
    cv::Mat T_w_r0 = pose2TransformationMatrix(robot->robotPose_w);
    Stg::Pose poseOther_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(otherRobot));
    // ModelPosition::Waypoint wp(poseOther_w, Color("green"));

    // ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
    return 0;
}

 int getOtherRobotPoseLaser(const Stg::ModelRanger::Sensor& sensor,  robot_t *robot, Stg::Pose &otherRobot)
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

        laser_x =  robot->robotPose.x +  scan[i] * cos(laser_orientation);
        laser_y =  robot->robotPose.y +  scan[i] * sin(laser_orientation);
        
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

Stg::Pose filterWithPriority(Stg::Pose* poseVector , int posNum)
{
  Stg::Pose avgDestinations= Stg::Pose(0,0,0,0);
  double devide = 1;
  for (int i=0 ; i<numberOfPosHist-1 ; i++)
  {
    devide *= 0.5;
    avgDestinations.x += poseVector[(posNum-i+ numberOfPosHist)%numberOfPosHist].x * devide;
    avgDestinations.y += poseVector[(posNum-i + numberOfPosHist)%numberOfPosHist].y * devide;
  }
  avgDestinations.x += poseVector[(posNum+numberOfPosHist-(numberOfPosHist-1))%numberOfPosHist].x * devide;
  avgDestinations.y += poseVector[(posNum + numberOfPosHist -(numberOfPosHist-1))%numberOfPosHist].y * devide;

  return avgDestinations;
}

Stg::Pose filterWithoutPriority(Stg::Pose* poseVector)
{
  Stg::Pose avgDestinations= Stg::Pose(0,0,0,0);
  for (int i=0 ; i<numberOfPosHist ; i++)
  {
    avgDestinations.x += poseVector[i].x;
    avgDestinations.y += poseVector[i].y;
  }
  avgDestinations.x = avgDestinations.x / (double)numberOfPosHist;
  avgDestinations.y = avgDestinations.y / (double)numberOfPosHist;
  return avgDestinations;
}


// inspect the ranger data and decide what to do
int laserCounter =0;
int getDestinationBasedOnOtherObjectPoses(robot_t *robot, Stg::Pose otherRobot, Stg::Pose& avgDestinations)
{
  int posNum = robot->TheposesOtherNum % numberOfPosHist;
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
  
  Stg::Pose historyAVG = filterWithoutPriority(robot->posesOther); 

  // double sumX = 0.0;
  // double sumY = 0.0;
  // double devider = 1;
  // for (int i=0; i<numberOfPosHist - 1 ; i++){
  //   devider *= 0.5;
  //   sumX += robot->posesOther[i].x * devider;
  //   sumY += robot->posesOther[i].y * devider; 
  // }
  // sumX += robot->posesOther[numberOfPosHist-1].x * devider;
  // sumY += robot->posesOther[numberOfPosHist-1].y * devider; 

  // double x = sumX;
  // double y = sumY;


  // double speedX = historyAVG.x - robot->posesAvgLastOther.x; 
  // double speedY = historyAVG.y - robot->posesAvgLastOther.y; 
  std::cout << "line 620" << std::endl;   
  double speedX = robot->kalmanFilter->state().at<float>(7,0)*.1; 
  double speedY = robot->kalmanFilter->state().at<float>(8,0)*.1;        
  std::cout << "line 623" << std::endl;   
  
  if (speedX == 0 && speedY == 0){
    speedX = robot->posesAvgLastOther.z;
    speedY = robot->posesAvgLastOther.a;
  }
  robot->posesAvgLastOther = Stg::Pose(historyAVG.x,historyAVG.y,speedX,speedY);
  robot->posesOther[posNum] = Stg::Pose(xTemp,yTemp,0,0);
  if (speedX*speedX + speedY*speedY == 0){   //to avoid devide by 0
    return 1;
  }
  double speedXU = speedX / sqrt(speedX*speedX + speedY*speedY);
  double speedYU = speedY / sqrt(speedX*speedX + speedY*speedY);
  robot->lastDestination[posNum] = Stg::Pose(historyAVG.x+speedX*2 + speedXU*followdist, historyAVG.y+speedY*2 + speedYU*followdist, 0.,0.);
  //-----------------------gitting dest history average focus on last destination---------

  avgDestinations = filterWithoutPriority(robot->lastDestination);

  // avgDestinations= Stg::Pose(0,0,0,0);
  // double devide = 1;
  // for (int i=0 ; i<numberOfPosHist-1 ; i++){
  //   devide *= 0.5;
  //   avgDestinations.x += robot->lastDestination[(posNum-i+ numberOfPosHist)%numberOfPosHist].x * devide;
  //   avgDestinations.y += robot->lastDestination[(posNum-i + numberOfPosHist)%numberOfPosHist].y * devide;

  // } 
  // avgDestinations.x += robot->lastDestination[(posNum+numberOfPosHist-(numberOfPosHist-1))%numberOfPosHist].x * devide;
  // avgDestinations.y += robot->lastDestination[(posNum + numberOfPosHist -(numberOfPosHist-1))%numberOfPosHist].y * devide;
  robot->TheposesOtherNum ++;
  return 0;
}

int FiterReferenceFrameUpdate(robot_t *robot)
{
  // transformation from frame 2 (current robot pose) to frame 1 (previous robot pose)
  cv::Mat T12 = pose2TransformationMatrix(robot->robotPose);
  // transformation from frame 2 to frame 1
  cv::Mat T21 = T12.inv();
  cv::Mat homogeneousVectorF1;
  cv::Mat homogeneousVectorF2;

  // for visualization purpose, plot other robot estimate wrt r0 (using its absolute position/orientation)
  // transformation from r0 to world frame
  cv::Mat T_w_r0 = pose2TransformationMatrix(robot->pos->GetPose());
  for (int i=0 ; i< numberOfPosHist ; i++){

    homogeneousVectorF1 = pose2HomgeneousVector(robot->lastDestination[i]);
    homogeneousVectorF2 = T21 * homogeneousVectorF1;
    robot->lastDestination[i] = homogeneousVector2Pose(homogeneousVectorF2);
    Stg::Pose poseOther_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(robot->lastDestination[i]));
    createWaypoint(robot, poseOther_w, "cyan");

    homogeneousVectorF1 = pose2HomgeneousVector(robot->posesOther[i]);
    homogeneousVectorF2 = T21 * homogeneousVectorF1;
    robot->posesOther[i] = homogeneousVector2Pose(homogeneousVectorF2);
    // std::cout << "filter reffrence points: " << robot->posesOther[i].x << " " << robot->posesOther[i].y;
    
    poseOther_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(robot->posesOther[i]));
    createWaypoint(robot, poseOther_w, "red");

  }


}



int PoseUpdate(Model *, robot_t *robot)
{
  // robot->robotPose = robot->pos->GetPose();
  // return 0;
  // std::cout << "velocity(x,y): " <<((Stg::Pose)robot->pos->GetVelocity()).x << " " << ((Stg::Pose)robot->pos->GetGlobalVelocity()).y << std::endl;
  if (!robot->isOtherPoseValid)
  {
    return 0;
  }

  cv::Mat state;
  if (robot->referenceLastUpdated >= numUpdateReferenceFrame)
  {
    ((ModelPosition*)robot->otherRobot)-> waypoints.clear();
    FiterReferenceFrameUpdate(robot);

    // transform the velocity to the new reference frame
    std::cout << "line 707" << std::endl;   
    cv::Mat oldState = robot->kalmanFilter->state();
    float oldVelOther_x = oldState.at<float>(7, 0);
    float oldVelOther_y = oldState.at<float>(8, 0);
    std::cout << "line 711" << std::endl;   
  
    float newVelOther_x = cos(robot->robotPose.a) * oldVelOther_x + sin(robot->robotPose.a) * oldVelOther_y;
    float newVelOther_y = -sin(robot->robotPose.a) * oldVelOther_x + cos(robot->robotPose.a) * oldVelOther_y;

    robot->robotPose = Stg::Pose(0, 0, 0, 0);
    robot->prevPoseMine = Stg::Pose(0, 0, 0, 0);
    robot->referenceLastUpdated = 0;

    // just for visualization of waypoints
    robot->robotPose_w = robot->pos->GetPose();

    // reset the filter
    cv::Mat newState = cv::Mat::zeros(9, 1, CV_32F);
    newState.at<float>(3, 0) = oldState.at<float>(3, 0);
    newState.at<float>(4, 0) = oldState.at<float>(4, 0);
    newState.at<float>(5, 0) = robot->relative_pose_other.x;
    newState.at<float>(6, 0) = robot->relative_pose_other.y;
    newState.at<float>(7, 0) = newVelOther_x;
    newState.at<float>(8, 0) = newVelOther_y;
    std::cout << "line 731" << std::endl;   
  
    robot->kalmanFilter->init(0, newState);
    std::cout << "line 734" << std::endl;   

    state = newState;
  }
  else
  {
    double omega_odom = ((Stg::Pose)robot->pos->GetVelocity()).a + (std::rand()/(double)RAND_MAX - 0.5)*sqrt(R1)*2;
    double v_odom = ((Stg::Pose)robot->pos->GetVelocity()).x + (std::rand()/(double)RAND_MAX - 0.5)*sqrt(R0)*2;

    double omega_cmd_vel = ((Stg::Pose)robot->pos->GetVelocity()).a + (std::rand()/(double)RAND_MAX - 0.5)*sqrt(Q4)*2;
    double v_cmd_vel = ((Stg::Pose)robot->pos->GetVelocity()).x + (std::rand()/(double)RAND_MAX - 0.5)*sqrt(Q3)*2;

    // measurement
    cv::Mat y = cv::Mat(4, 1, CV_32F);
    y.at<float>(0, 0) = v_odom;
    y.at<float>(1, 0) = omega_odom;
    y.at<float>(2, 0) = robot->relative_pose_other.x + (std::rand()/(double)RAND_MAX - 0.5)*sqrt(R2)*2;
    y.at<float>(3, 0) = robot->relative_pose_other.y + (std::rand()/(double)RAND_MAX - 0.5)*sqrt(R3)*2;

    // control input
    cv::Mat u = cv::Mat(2, 1, CV_32F);
    u.at<float>(0, 0) = v_cmd_vel;
    u.at<float>(1, 0) = omega_cmd_vel;
    std::cout << "line 759" << std::endl;   

    if (!robot->kalmanFilter->isInitialized())
    {
      cv::Mat initState = cv::Mat::zeros(NUM_STATES, 1, CV_32F);
      initState.at<float>(5, 0) = robot->poseOtherPrev.x;
      initState.at<float>(6, 0) = robot->poseOtherPrev.y;
      robot->kalmanFilter->init(0, cv::Mat::zeros(NUM_STATES, 1 ,CV_32F));
    }
    
    robot->kalmanFilter->update(y, 0.1, u);

    state = robot->kalmanFilter->state();
    std::cout << "line 772" << std::endl;   

    robot->robotPose.x = state.at<float>(0, 0);
    robot->robotPose.y = state.at<float>(1, 0);
    robot->robotPose.a = state.at<float>(2, 0);
    robot->robotPose.a = atan2(sin(robot->robotPose.a), cos(robot->robotPose.a));

    robot->prevPoseMine = robot->robotPose;
    
    // double omega = ((Stg::Pose)robot->pos->GetVelocity()).a;
    // double v = ((Stg::Pose)robot->pos->GetVelocity()).x;

    // std::cout << "Y velocity: " << ((Stg::Pose)robot->pos->GetVelocity()).y << std::endl;

    // robot->robotPose.x +=  v * cos(robot->prevPoseMine.a) / 10. ;
    // robot->robotPose.y +=  v * sin(robot->prevPoseMine.a) / 10.;
    
    // robot->prevPoseMine = robot->robotPose;
    // robot->robotPose.a += omega / 10. ;
    // robot->robotPose.a = atan2(sin(robot->robotPose.a), cos(robot->robotPose.a));
  }

 
  robot->referenceLastUpdated++;

  // for visualization purpose, plot other robot estimate wrt r0 (using its absolute position/orientation)
  // transformation from r0 to world frame
  cv::Mat T_w_r0 = pose2TransformationMatrix(robot->robotPose_w);
  Stg::Pose robotPose_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(robot->robotPose));
  ModelPosition::Waypoint wp(robotPose_w, Color("yellow"));
  ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);

  cv::Mat otherRobotPose_r0(3, 1, CV_32F);
  otherRobotPose_r0.at<float>(0, 0) = state.at<float>(5, 0);
  otherRobotPose_r0.at<float>(1, 0) = state.at<float>(6, 0);
  otherRobotPose_r0.at<float>(2, 0) = 1;

  Stg::Pose avgDestinations;
  if (getDestinationBasedOnOtherObjectPoses(robot, Stg::Pose(state.at<float>(5, 0),state.at<float>(6, 0),0,0), avgDestinations)) // will set avg destination
    return 0;
  setSpeed(robot, avgDestinations);

  
  ModelPosition::Waypoint wp2(homogeneousVector2Pose(T_w_r0 * otherRobotPose_r0), Color("green"));
  ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp2);
  
  return 0;
}

int createWaypoint(robot_t *robot, Stg::Pose pose, char* color)
{
    ModelPosition::Waypoint wp(Stg::Pose(pose.x ,pose.y , 0.0, pose.a), Color(color));
   ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
}

int LaserUpdate(Model *, robot_t *robot)
{
  if (!UseLaser)
    return 0;


  Stg::Pose otherRobot;
  if (getOtherRobotPoseLaser (robot->laser->GetSensors()[0], robot, otherRobot)) // no position found
    return 0;

  Stg::Pose avgDestinations;
  if (getDestinationBasedOnOtherObjectPoses(robot, otherRobot, avgDestinations)) // will set avg destination
    return 0;
  setSpeed(robot, avgDestinations);

  return 0;
 }

int dummyCounter=0;

int setSpeed(robot_t *robot, Stg::Pose destination)
{
  // for visualization purpose, plot other robot estimate wrt r0 (using its absolute position/orientation)
  // transformation from r0 to world frame
  cv::Mat T_w_r0 = pose2TransformationMatrix(robot->robotPose_w);
  Stg::Pose destination_w = homogeneousVector2Pose(T_w_r0 * pose2HomgeneousVector(destination));
  ModelPosition::Waypoint wp(destination_w ,Color("blue"));
  ((ModelPosition*)robot->otherRobot)-> waypoints.push_back(wp);
  
  if ((destination.x - robot->robotPose.x)==0) // Avoid devide by 0
    return 0;
  double m = (destination.y - robot->robotPose.y) / (destination.x - robot->robotPose.x);
  double degreeD = atan(m);

  if (destination.x < robot->robotPose.x && destination.y > robot->robotPose.y  )
    degreeD = M_PI + degreeD;  
  else if (destination.x < robot->robotPose.x && destination.y < robot->robotPose.y  )
    degreeD = -M_PI + degreeD;

  if (abs(degreeD -robot->degreeD) >= 180){
    // std::cout << "degreed + .: " << degreeD -robot->degreeD <<std::endl;
    degreeD = robot->degreeD;
  }
  robot->degreeD = degreeD;
  // std::cout << "m: "<< m << "atan: " <<  degreeD*180 / M_PI << std::endl;
  
  double Dturn = robot->robotPose.a - degreeD;

  if (Dturn >= M_PI)
    Dturn -= M_PI*2;
  else if (Dturn <= -M_PI)
     Dturn += M_PI*2;
  // std::cout << "dturn: " << Dturn << std::endl;
  //set turn speed
  double Tspeed = robot->pidturn.calculate( 0 ,  Dturn , 0.01 );
  // std::cout << "Tspeed: " << Tspeed << std::endl;
  robot->pos->SetTurnSpeed(Tspeed);

  // dummyCounter++;

  // if (dummyCounter%2)
  //   robot->pos->SetTurnSpeed(
  //     0.4
  //   );
  // else{
  //   robot->pos->SetTurnSpeed(
  //     -0.4
  //   );
  // }

  robot->distance = sqrt(pow(destination.x- robot->robotPose.x, 2) +  pow(destination.y - robot->robotPose.y, 2));
  // std::cout<< "distance: " << robot->distance << std::endl;
  double Xspeed = robot->pidcruse.calculate(0 ,robot->distance, 0.01 );
  // std::cout << "Xspeed: " << Xspeed << std::endl;
  robot->pos->SetXSpeed(Xspeed);
  // robot->pos->SetXSpeed(0.4);

  return 0; // run again
}