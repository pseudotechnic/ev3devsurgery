#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265

//using namespace std;
//using namespace ev3dev;

//initialize methods
double radToDeg(double x);
void angleCalc(double pX, double pY, double pZ, double ang, int* answer);
void posUpdate(std::string motor, std::string var);

class Motor {
public:
  Motor(const char* port);
  ~Motor() {};

  //void initMotors();
  void   Move(int target);
  double GetPosition();
  void   ResetPosition(double x);
  void   ErrorPrintLine(const char* str);

protected:
  int  GetMotorPath(const char* port, std::string& path);
  bool OpenOutStream(const char* command, std::ofstream& os);
  bool OpenInStream(const char* command, std::ifstream& os);

  std::string motorPath;
  std::string motorPort;

  std::ofstream duty_cycle_sp;
  std::ofstream speed_sp; 
  std::ofstream time_sp;
  std::ofstream position_sp;
  std::ofstream command;
  std::ofstream position;
  std::ofstream speed_regulation;
  std::ofstream stop_command;

  std::ifstream in_position;
};


Motor::Motor(const char* port)
{

  this->motorPort = port;

  GetMotorPath(port, this->motorPath);

  std::string commandPath;
  commandPath = this->motorPath + "/stop_command";
  stop_command.open(commandPath.c_str());
  stop_command << "hold" << std::endl;
  stop_command.close();
}


int Motor::GetMotorPath(const char* port, std::string& path)
{
  std::string basePath = "/sys/class/tacho-motor/";

  for (int i = 0; i < 20; i ++)
    {
      std::stringstream ss;
      ss << basePath << "motor" << i << "/address";
      std::string addressPath = ss.str();
      std::ifstream addressFile(addressPath.c_str());

      std::cout << "Checking " << addressPath << std::endl;
      
      if (addressFile.good())  // if the file exists
	{
	  std::string content;
	  addressFile >> content;
	  
	  if (content == port)
	    {
	      std::stringstream ss;
	      ss << basePath << "motor" << i << "/";
	      path = ss.str();
	      return 1;
	    }
	  addressFile.close();
	}
	    
    }
  return 0;
}


void Motor::ErrorPrintLine(const char* str)
{
  std::cerr << motorPort << " >> " << str << std::endl;
}


bool Motor::OpenOutStream(const char* command, std::ofstream& os)
{
  std::string commandPath;
  commandPath = this->motorPath+"/"+command;
  os.open(commandPath.c_str());
  if(!os.is_open())
    {
      std::cerr <<"failed to open file: " << commandPath << std::endl;
      return false;
    }
  return true;
}

bool Motor::OpenInStream(const char* command, std::ifstream& is)
{
  std::string commandPath;
  commandPath = this->motorPath+"/"+command;
  is.open(commandPath.c_str());
  if(!is.is_open())
    {
      std::cerr <<"failed to open file: " << commandPath << std::endl;
      return false;
    }
  return true;
}

void Motor::Move(int target)
{
  std::cerr << "Moving motor " << this->motorPort << " to target " << target << std::endl;
  
  std::string path; 
  OpenOutStream("speed_sp", speed_sp);
  OpenOutStream("position_sp", position_sp);
  OpenOutStream("command", command);
  OpenOutStream("position", position);
  OpenOutStream("speed_regulation", speed_regulation);
  
  std::cout <<"enabling speed regulation" <<std::endl;
  speed_regulation << "on" << std::endl;
  std::cout <<"Set speed." <<std::endl;
  speed_sp << 500 << std::endl;
  std::cout << "Set position." << std::endl;
  position_sp << target << std::endl;
  std::cout << "Set command." << std::endl;
  command << "run-to-abs-pos" << std::endl;
  
  sleep(5);
  
  std::cout << "resetting motors" << std::endl; 
  command << "reset" << std::endl;				
  
  speed_regulation.close();
  speed_sp.close();
  position_sp.close();
  position.close();
  command.close();
  
}

double Motor::GetPosition()
{
  OpenInStream("position", in_position);
  int value;
  in_position >> value;
  in_position.close();
  return (double) value;
}

void Motor::ResetPosition(double x)
{
  OpenOutStream("position", position);
  std::string path; 
  int intx = round(x);
  position << intx << std::endl;
  position.close();
}


class GearMotor: public Motor
{
public:
  enum {
    JOINT_ARM,
    JOINT_FOREARM,
    JOINT_FINGER
  };
  
public:
  GearMotor(const char* port) : Motor(port) {};
  ~GearMotor() {};

  bool Switch(int joint);
};

bool GearMotor::Switch(int joint)
{

  int angle;

  switch (joint)
    {
    case JOINT_ARM:
      angle = 75;
      std::cerr << "Switch to JOINT_ARM" << std::endl;
      break;
    case JOINT_FOREARM:
      angle = 165;
      std::cerr << "Switch to JOINT_FOREARM" << std::endl;
     break;
    case JOINT_FINGER:
      angle = 23;
      std::cerr << "Switch to JOINT_FINGER" << std::endl;
      break;
    default:
      angle = 0;
      break;
    }

  this->Move(angle);
  sleep(1);
}


int driveM(double x, double y, double z, double ang, Motor& roll, Motor& drive, GearMotor& gear);

//length of arms
int L1 = 9.5;
int L2 = 20;
int L3 = 9;

//these are the current angles. It is set to 0 because assume robot starts at 0 position
int angArm;
int angForearm;
int angFinger;

int main(int argc, char* argv[])
{
  double xCoord= 00;
  double yCoord= 10; 
  double zCoord= 20; 

  if (argc > 3)
    {
    xCoord= atoi(argv[1]);
    yCoord= atoi(argv[2]); 
    zCoord= atoi(argv[3]); 
    }

  Motor roll("outA");
  Motor drive("outB");
  GearMotor gear("outC");

  double angle= 90;

  // Homing
  roll.ResetPosition(0);
  drive.ResetPosition(0);
  gear.ResetPosition(0);

  angArm = 90;
  angForearm = 0;
  angFinger = 0;

  driveM(xCoord,yCoord,zCoord,angle, roll, drive, gear);
}


//radian to degree converter
double radToDeg(double x)
{
  //double answer = tan(x * PI / 180);
  double answer = x * 180.0 / PI;
  return answer; 
}


double calcRollAngle(double pX, double pY, double pZ)
{
  return (90.0 - radToDeg(atan2(pY, pX)));
}
	
double transformTargetToInPlane(double pX, double pY, double pZ, double& pinX, double& pinY)
{
  pinX = pZ;
  pinY = sqrt(pX*pX + pY*pY);
}


//calculate the angles
void angleCalcInPlane(double pX, double pY, double pZ, double ang, int* answer)
   {		
     //double pX = L1*cos(angle1) + L2*cos(angle1 + angle2);
     //double pY = L2*sin(angle1) + L2*sin(angle1 + angle2); 

     double pXF = pX + L3*ang; 
     double pXY = pY + L3*ang; 
		
     int angle1 = ceil(radToDeg((0.5*asin((pX*pX + pY*pY - L1*L1)/(L2*L2)))-acos((pX*pX + pY*pY - (L1*L1 + L2*L2))/(2*L1*L2))));
     int angle2 = ceil(radToDeg(acos((pX*pX + pY*pY - (L1*L1 + L2*L2))/(2*L1*L2))));
     int angle3 = ceil(radToDeg(ang - (0.5*asin((pX*pX + pY*pY - L1*L1)/(L2*L2)))));
		
     answer[0] = angle1;
     answer[1] = angle2;
     answer[2] = angle3;
   }
   
				
//drive to the listed position inverse kinematics
//drive(10,10,0,90)
int driveM(double x, double y, double z, double ang, Motor& roll, Motor& drive, GearMotor& gear)
{

  int angleData[4];

  // Roll angle
  angleData[3] = calcRollAngle(x, y, z);

  double pinX;
  double pinY;

  transformTargetToInPlane(x, y, z, pinX, pinY);

  angleCalcInPlane(pinX,pinY,0.0, ang, angleData);
  std::cerr << "angleData = (" 
	    << angleData[0] << ", "
	    << angleData[1] << ", "
	    << angleData[2] << ")"
	    << std::endl;

  // -45 deg for arm -> 1800 deg for drive  -> -40 deg / deg
  double ratioArm = -40.0;
  // -90 deg for forearm -> 1500 deg for drive
  double ratioForearm = 16.6666;
  // 90 deg for finger -> 270 deg for drive
  double ratioFinger = 3.0;

  gear.Switch(GearMotor::JOINT_ARM);
  drive.ResetPosition(0);
  drive.Move((double)(angleData[0]-angArm)*ratioArm);
  angArm += drive.GetPosition()/ratioArm;

  gear.Switch(GearMotor::JOINT_FOREARM);
  drive.ResetPosition(0);
  drive.Move((double)(angleData[1]-angForearm+angArm)*ratioForearm);
  angForearm += drive.GetPosition()/ratioForearm-angArm;

  gear.Switch(GearMotor::JOINT_FINGER);
  drive.ResetPosition(0);
  drive.Move((double)(angleData[2]-angFinger+angArm+angForearm)*ratioFinger);
  angFinger += drive.GetPosition()/ratioFinger-angArm-angForearm;

  roll.Move((double)angleData[3]);
  
} 
	
	
//Posgear not updating correctly??
