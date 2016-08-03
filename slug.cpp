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

	std::ofstream duty_cycle_sp;
    std::ofstream speed_sp; 
	std::ofstream time_sp;
	std::ofstream position_sp;
	std::ofstream command;
	std::ofstream position;
    std::ofstream speed_regulation;
	std::ofstream stop_command;

	std::ifstream in_position;
	
	std::string roll;
	std::string drive;
	std::string gear;
	
//current position of different motors... unnecessary?
	int posRoll;
	int posGear;
	int posDrive;
	
//length of arms
	int L1 = 11;
	int L2 = 20;
	int L3 = 9.5;
	
//these are the current angles. It is set to 0 because assume robot starts at 0 position
	int ang1;
	int ang2;
	int ang3;

//initialize methods
void initMotors();
int GetMotorPath(const char* port, std::string& path);
double radToDeg(double x);
void angleCalc(double pX, double pY, double pZ, double ang, int* answer);
void motorMove(string motor, int target);	
void initDrivePosUpdate(string var);	
void posUpdate(string motor, string var)
int driveM(double x, double y, double z, double ang);        

int main()
{
  GetMotorPath("outA", roll);
  GetMotorPath("outB", drive);
  GetMotorPath("outC", gear);

  ang1 = 0;
  ang2 = 0;
  ang3 = 0;
  
  double xCoord= 10;
  double yCoord= 10; 
  double zCoord= 0; 
  double angle= 90;
  initMotors();
  driveM(xCoord,yCoord,zCoord,angle);
}

int GetMotorPath(const char* port, std::string& path)
{
  std::string basePath = "/sys/class/tacho-motor/";

  for (int i = 0; i < 4; i ++)
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

void initMotors()
{
	
	std::string path;
	path = roll + "/stop_command";
	stop_command.open(path.c_str());
	stop_command << "brake" << std::endl;
	stop_command.close();

	path = drive + "/stop_command";
	stop_command.open(path.c_str());
	stop_command << "brake" << std::endl;
	stop_command.close();
	
	path = gear + "/stop_command";
	stop_command.open(path.c_str());
	stop_command << "brake" << std::endl;
	stop_command.close(); 
}

//radian to degree converter
double radToDeg(double x)
{
    double answer = tan(x * PI / 180);
    return answer; 
  }
	
//calculate the angles
void angleCalc(double pX, double pY, double pZ, double ang, int* answer)
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
   
				
void motorMove(string motor, int target)				
{
	std::string path; 
	path = motor+"/speed_sp";
	speed_sp.open(path.c_str());
	if(!speed_sp.is_open())
	{
		std::cerr <<"failed to open file: " << path << std::endl;
	}
	path = motor+"/position_sp";
	position_sp.open(path.c_str());
	if (!position_sp.is_open())
	{
		std::cerr << "Failed to open file: " << path << std::endl;
	}
	path = motor+"/command";
	command.open(path.c_str());
	if (!command.is_open())
	{
		std::cerr << "Failed to open file: " << path << std::endl;
	}
	path = motor+"/position";
	position.open(path.c_str());
	if (!position.is_open())
	{
		std::cerr << "Failed to open file: " << path << std::endl;
	}
	path = motor+"/speed_regulation";
	speed_regulation.open(path.c_str());
	if (!speed_regulation.is_open())
	{
		std::cerr << "Failed to open file: " << path << std::endl;
	}
	
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
				
	std::cout << "target location: " << angleData[0] << ", " << angleData[1] << ", " << angleData[2] << std::endl;
	std::cout << "actual location: " << ang1 << ", " << ang2 << ", " << ang3 << std::endl;
	std::cout <<"posGear: " << posGear << std::endl; 
}

//takes current ang (ang1, ang2, ang3)			
void initDrivePosUpdate(string var)				
{
	std::string path; 
	path = drive+"/position";
	position.open(path.c_str());
	position << var << std::endl;
}			

//takes current ang (ang1, ang2, ang3)
void posUpdate(string motor, string var)
{
	std::string path; 
	path = motor + "/position";
	std::string stringPos;
	in_position.open(path.c_str());
	int value;
	in_position >> value;
	var = value; 
	n_position.close();
}
	
	//drive to the listed position inverse kinematics
	//drive(10,10,0,90)
	int driveM(double x, double y, double z, double ang)
	{		
		int angleData[3];
		angleCalc(x,y,z,ang, angleData);
		std::cerr << "angleData = (" 
			  << angleData[0] << ", "
			  << angleData[1] << ", "
			  << angleData[2] << ")"
			  << std::endl;
			
		int loopCount = 0;
		do{	  
		
			std::string path;
			
			path = gear + "/position";
			std::string stringPos;
			in_position.open(path.c_str());
			int value;
			in_position >> value;
			posGear = value; 
			in_position.close();				
			
			loopCount++;
			
			//check which gear it's on
			if(abs(posGear - 75) < 5) //arm <10
			{
				std::cout<< "Gear is driving main arm."<<std::endl;
				//11cm
				
				//drive
				initDrivePosUpdate(ang1);
				motorMove(drive, angleData[0]);
				posUpdate(drive, ang1);
				
				//gears
				motorMove(gear, 165);
				posUpdate(gear, posGear);
				
			}	
			else if(abs(posGear - 165) < 5) //forearm < 15
			{ 
				std::cout<< "Gear is driving forearm."<<std::endl;
				//measure degrees the forearm moves when reached the tacho value: 7915
				//use ratio between them to figure out location of arm
				//20cm
					
				//drive
				initDrivePosUpdate(ang2);
				motorMove(drive, angleData[1]);
				posUpdate(drive, ang2);
;
				/*
				path = gear + "/position";
				std::string stringPos;
				in_position.open(path.c_str());
				int value;
				in_position >> value;
				posGear = value; 
				in_position.close();*/	

				//gears
				motorMove(gear, 23);
				posUpdate(gear, posGear)

			}	
			else if(abs(posGear-23) < 5) //finger  <23
			{
				std::cout<< "Gear is driving finger."<<std::endl;
				//9.5cm
		
				//drive
				initDrivePosUpdate(ang3);
				motorMove(drive, angleData[2]);
				posUpdate(drive, ang3);
				
				/*
				path = gear + "/position";
				std::string stringPos;
				in_position.open(path.c_str());
				//in_position >> stringPos;
				//int value = atoi(stringPos.c_str());
				int value;
				in_position >> value;
				posGear = value; 
				in_position.close();*/ //fix later
				
				//gear	
				motorMove(gear, 75);
				posUpdate(gear, posGear);

			}
			else
			{
				std::cout<< "Gear is still shifting, please wait."<<std::endl;
				motorMove(gear, 75);
				posUpdate(gear, posGear);
			}	
		} while(angleData[0]!=ang1 || angleData[1]!= ang2 || angleData[2] !=ang3);		
		} 
	
	
//Posgear not updating correctly??
