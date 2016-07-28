//#include "ev3dev.h"

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265

//using namespace std;
//using namespace ev3dev;

	std::ofstream duty_cycle_sp;
	std::ofstream time_sp;
	std::ofstream position_sp;
	std::ofstream command;
	std::ofstream position;
std::ofstream speed_regulation_enabled; 
	
	std::string roll ="/sys/class/tacho-motor/motor0";
	std::string drive = "/sys/class/tacho-motor/motor1";
	std::string gear = "/sys/class/tacho-motor/motor2";
	
//current position of different motors... unnecessary?
	int posRoll;
	int posGear;
	int posDrive;
	
//length of arms
	int L1 = 11;
	int L2 = 20;
	int L3 = 9.5;
	
//these are the current angles. It is set to 0 because assume robot starts at 0 position
	int ang1 = 0;
	int ang2 = 0;
	int ang3 = 0;

//initialize methods
double radToDeg(double x);
void angleCalc(double pX, double pY, double pZ, double ang, int* answer);
int driveM(double x, double y, double z, double ang);        
	
	int main()
	{
	  double xCoord= 10;
	  double yCoord= 10; 
	  double zCoord= 0; 
	  double angle= 90;
		driveM(xCoord,yCoord,zCoord,angle);
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
	
	//drive to the listed position inverse kinematics
	//drive(10,10,0,90)
	int driveM(double x, double y, double z, double ang)
	{		
		int angleData[3];
		angleCalc(x,y,z,ang, angleData);
			//check which gear it's on
			if(abs(posGear - 75) < 5) //arm <10
			{
				std::cout<< "Gear is driving main arm."<<std::endl;
				//11cm
					
				//open drive
				std::string path;
				path = drive+"/duty_cycle_sp";
				duty_cycle_sp.open(path.c_str() ,std::ofstream::out);
				path = drive+"/position_sp";
				position_sp.open(path.c_str(), std::ofstream::out);
				path = drive+"/command";
				command.open(path.c_str() ,std::ofstream::out);
				path = drive+"/position";
				position.open(path.c_str());
				
				position << ang1 << std::endl;
	
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << angleData[0] << std::endl;
				std::cout << "Set command." << std::endl;
				command << "run_to_abs_pos" << std::endl;
	
				sleep(3);
				ang1 = angleData[0];
					
				//open gears
				path = gear+"/duty_cycle_sp";
				duty_cycle_sp.open(path.c_str(),std::ofstream::out);
				path = gear+"/position_sp";
				position_sp.open(path.c_str(), std::ofstream::out);
				path = gear+"/command";
				command.open(path.c_str(),std::ofstream::out);
				path = gear+"/position";
				position.open(path.c_str());
	
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << 165 << std::endl;
				std::cout << "Set command." << std::endl;
				command << "run_to_abs_pos" << std::endl;
			}	
			else if(abs(posGear - 165) < 5) //forearm < 15
			{ 
				std::cout<< "Gear is driving forearm."<<std::endl;
				//measure degrees the forearm moves when reached the tacho value: 7915
				//use ratio between them to figure out location of arm
				//20cm
					
				//open drive
				std::string path;
				path = drive+"/duty_cycle_sp";
				duty_cycle_sp.open(path.c_str(),std::ofstream::out);
				path = drive+"/position_sp";
				position_sp.open(path.c_str(), std::ofstream::out);
				path = drive+"/command";
				command.open(path.c_str(), std::ofstream::out);
				path = drive+"/position";
				position.open(path.c_str());
					
				position << ang2 << std::endl;

				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << angleData[1] << std::endl;
				std::cout << "Set command." << std::endl;
				command << "run_to_abs_pos" << std::endl;
		
				sleep(3);
				ang2 = angleData[1];	

				//open gears
				path = gear+"/duty_cycle_sp";
				duty_cycle_sp.open(path.c_str(),std::ofstream::out);
				path = gear+"/position_sp";
				position_sp.open(path.c_str(), std::ofstream::out);
				path = gear+"/command";
				command.open(path.c_str(),std::ofstream::out);
				path = gear+"/position";
				position.open(path.c_str());
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << 23 << std::endl;
				std::cout << "Set command." << std::endl;
				command << "run_to_abs_pos" << std::endl;					
			}	
			else if(abs(posGear-23) < 5) //finger  <23
			{
				std::cout<< "Gear is driving finger."<<std::endl;
				//9.5cm
					
				//open drive
				std::string path;
				path = gear+"/duty_cycle_sp";
				duty_cycle_sp.open(path.c_str(),std::ofstream::out);
				path = gear+"/position_sp";
				position_sp.open(path.c_str(), std::ofstream::out);
				path = gear+"/command";
				command.open(path.c_str(),std::ofstream::out);
				path = gear+"/position";
				position.open(path.c_str());
				path = gear+"/speed_regulation_enabled";
				speed_regulation_enabled.open(path.c_str(),std::ofstream::out);
				
				position << ang3 << std::endl;
	
				std::cout <<"Enabling Speed Regulation" <<std::endl;
				speed_regulation_enabled <<"on"<< std::endl;
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << angleData[2] << std::endl;
				std::cout << "Set command." << std::endl;
				command << "run_to_abs_pos" << std::endl;
		
				sleep(3);
				ang3 = angleData[2];
					
				//open gears
				path = gear+"/duty_cycle_sp";
				duty_cycle_sp.open(path.c_str(),std::ofstream::out);
				path = gear+"/position_sp";
				position_sp.open(path.c_str(), std::ofstream::out);
				path = gear+"/command";
				command.open(path.c_str(),std::ofstream::out);
				path = gear+"/position";
				position.open(path.c_str());
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << 75 << std::endl;
				std::cout << "Set command." << std::endl;
				command << "run_to_abs_pos" << std::endl;
			}
			else
			{
				std::cout<< "Gear is still shifting, please wait."<<std::endl;
				std::string path; 
					//open gears
				path = gear+"/duty_cycle_sp";
				duty_cycle_sp.open(path.c_str(),std::ofstream::out);
				path = gear+"/position_sp";
				position_sp.open(path.c_str(), std::ofstream::out);
				path = gear+"/command";
				command.open(path.c_str(),std::ofstream::out);
				path = gear+"/position";
				position.open(path.c_str());

				path = gear +"/speed_regulation_enabled";
				speed_regulation_enabled.open(path.c_str(), std::ofstream::out);
		
				std::cout <<"enabling speed regulation" <<std::endl;
				speed_regulation_enabled << "on" <<std::endl;
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << 75 << std::endl;
				std::cout << "Set command." << std::endl;
				command << "run_to_abs_pos" << std::endl;

			}						
		} 
	
	
