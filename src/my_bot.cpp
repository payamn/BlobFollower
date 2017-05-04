#include <limits> include <vector> include <iostream> include <cassert>
#include <algorithm>
#include <cmath>
#include "pid.h"

#include "my_bot.h"


using namespace Stg;

static const double followdist = 1.4;
static const double cruisespeed = 0.4;
static const double avoidspeed = 0.05;
static const double avoidturn = 0.5;
static const double minfrontdistance = 1.0; // 0.6
static const double stopdist = 0.3;
static const int avoidduration = 10;
static const int maxblobx = 79;
static bool isahead = true;
typedef struct {
  ModelPosition *pos;
  ModelRanger *laser;
  ModelBlobfinder *blob;
  double lastposeother;
  int state; // 1 means blob detected, 0 not detected
  PID pidcruse;
  PID pidturn;
  int avoidcount, randcount;
} robot_t;


int counter = 0;
int BlobUpdate(Model *, robot_t *robot);
int LaserUpdate(Model *mod, robot_t *robot);

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
    robot->pidturn.set(avoidturn, -avoidturn, 0.04, 0.0005, 0.000);
    robot->pidcruse.set( cruisespeed,0 , cruisespeed*4, 0.001, 0.001);
  }
  else {
    robot->pidturn.set(avoidturn, -avoidturn, 0.04, 0.0005, 0.000);
    robot->pidcruse.set(0, cruisespeed, cruisespeed*4, 0.001, 0.001);
  }
  robot->avoidcount = 0;
  robot->lastposeother = 0;
  robot->randcount = 0;
  robot->state = 0;
  robot->pos = dynamic_cast<ModelPosition *>(mod);
  if (!robot->pos) {
    PRINT_ERR("No position model given in wander controller.");
    exit(1);
  }

  
  robot->pos->Subscribe(); // starts the position updates

  ModelRanger *first_laser = dynamic_cast<ModelRanger *>(robot->pos->GetChild("ranger:0")),
              *second_laser = dynamic_cast<ModelRanger *>(mod->GetChild("ranger:1"));
  if (!first_laser || !second_laser) {
    PRINT_ERR1("Wander controller requires 2 range sensors (%u given).",
               unsigned(bool(first_laser) + bool(second_laser)));
    exit(2);
  }
  first_laser->Subscribe();
  robot->laser = second_laser;
  robot->laser->AddCallback(Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
  robot->laser->Subscribe(); // starts the ranger updates

  robot->blob = dynamic_cast<ModelBlobfinder *>(robot->pos->GetChild("blobfinder:0"));//(robot->pos->GetWorld()->GetModel("blob"));
  if (!robot->blob)
    std::cout << "error" << "\n";
  robot->blob->AddCallback(Model::CB_UPDATE, model_callback_t(BlobUpdate), robot);
  robot->blob->Subscribe();
  return 0; // ok
}

int BlobUpdate(Model *, robot_t *robot)
{
  counter++;
  // no blob
  if (!robot->blob->GetBlobs().size() ){
    robot->state = 0;
    return 0;
  }
  robot->state = 1;
  double range = robot->blob->GetBlobs()[0].range;
  int centerPointX = (robot->blob->GetBlobs()[0].left + robot->blob->GetBlobs()[0].right) /2;
  double degree = -asin(1.0 / sqrt(3) * (2.0 * centerPointX / 79 - 1));

  std::cout <<  maxblobx /2 - centerPointX << " range:" << range <<"\n" 
            // <<  robot->blob->GetBlobs()[0].left<< " right:" 
            // << robot->blob->GetBlobs()[0].right  <<  " " 
            // << robot->blob->GetBlobs()[0].range << " "  
            << "pose main: " << robot->pos->GetPose().x << " " << robot->pos->GetPose().y << " rotation:" << robot->pos->GetPose().a * 180 / M_PI << "\n"               
            << "pose secn: " << robot->pos->GetPose().x + range*sin(degree) << " " << robot->pos->GetPose().y - range*cos(degree) << " rotation:" << robot->pos->GetPose().a * 180 / M_PI << "\n" 
            << degree
            << std::endl;
  


  return 0;
  //set turn speed'
  robot->pos->SetTurnSpeed(
    robot->pidturn.calculate( maxblobx /2,  centerPointX , 0.1 )
  );

  robot->lastposeother = centerPointX ;
  // if (centerPointX < maxblobx /2 ){ 
  //   std::cout << "left" << std::endl;
  //   robot->pos->SetTurnSpeed((avoidturn/10.0)*(maxblobx /2 - centerPointX));
  // }
  // else if (centerPointX > maxblobx /2 ){ 
  //   std::cout << "right" << std::endl;
  //   robot->pos->SetTurnSpeed(-(avoidturn/10.0)*(centerPointX-maxblobx /2 ));
  // }
  // else{
  //   robot->pos->SetTurnSpeed(0);
  // }
  // set forward speed
  // if (robot->blob->GetBlobs()[0].range >  followdist){
    robot->pos->SetXSpeed(
    robot->pidcruse.calculate(followdist ,robot->blob->GetBlobs()[0].range, 0.1 )
    );
    // robot->pos->SetXSpeed((cruisespeed*4)*( robot->blob->GetBlobs()[0].range) - followdist);
  // }
  // else{
  //   std::cout << "speed 0" << std::endl;
  //   robot->pos->SetXSpeed(0);
  // }

  return 0; // run again
}
// inspect the ranger data and decide what to do
int LaserUpdate(Model *, robot_t *robot)
{
  return 0;

  if (robot->state)
    return 0;
  // get the data
  const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
  uint32_t sample_count = scan.size();
  if (sample_count < 1)
    return 0;

  bool obstruction = false;
  bool stop = false;

  // find the closest distance to the left and right and check if
  // there's anything in front
  double minleft = 1e6;
  double minright = 1e6;

  for (uint32_t i = 0; i < sample_count; i++) {

    if ((i > (sample_count / 3)) && (i < (sample_count - (sample_count / 3)))
        && scan[i] < minfrontdistance) {
      obstruction = true;
    }

    if (scan[i] < stopdist) {
      stop = true;
    }

    if (i > sample_count / 2)
      minleft = std::min(minleft, scan[i]);
    else
      minright = std::min(minright, scan[i]);
  }

 
  if (obstruction || stop || (robot->avoidcount > 0)) {
    robot->pos->SetXSpeed(stop ? 0.0 : avoidspeed);

    /* once we start avoiding, select a turn direction and stick
 with it for a few iterations */
    if (robot->avoidcount < 1) {
      robot->avoidcount = random() % avoidduration + avoidduration;

      if (minleft < minright) {
        robot->pos->SetTurnSpeed(-avoidturn);
      } else {
        robot->pos->SetTurnSpeed(+avoidturn);
      }
    }

    robot->avoidcount--;
  } else {

    robot->avoidcount = 0;
    robot->pos->SetXSpeed(cruisespeed);
    //robot->pos->SetTurnSpeed(0);
  }

  //  if( robot->pos->Stalled() )
  //     {
  //        robot->pos->SetSpeed( 0,0,0 );
  //        robot->pos->SetTurnSpeed( 0 );
  // }

  return 0; // run again
}

