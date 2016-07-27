//#include "ev3dev.h"

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#define PI 3.14159265

//using namespace std;
//using namespace ev3dev;

	std::ofstream duty_cycle_sp;
	std::ofstream time_sp;
	std::ofstream position_sp;
	std::ofstream command;
	
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
int* angleCalc(double x, double y, double z, double ang);
int driveM(double x, double y, double z, double ang);        
	
	int main()
	{
	  double xCoord= 10;
	  double yCoord= 10; 
	  double zCoord= 0; 
	  double angle= 90;
		drive(xCoord,yCoord,zCoord,angle);
	}

	//radian to degree converter
	double radToDeg(double x)
	{
		double answer = tan(x * PI / 180);
		return answer; 
	}
	
	//calculate the angles
	int* angleCalc(double x, double y, double z, double ang)
	{		
		double pXF = pX + L3*ang; 
		double pXY = pY + L3*ang; 
		
		double pX = L1*cos(angle1) + L2*cos(angle1 + angle2);
		double pY = L2*sin(angle1) + L2*sin(angle1 + angle2); 
		
		int angle1 = ceil(radToDeg((0.5*asin((pX^2 + pY^2 - L1^2)/L2^2))-acos((pX^2 + pY^2 - (L1^2 + L2^2))/2*L1*L2)));
		int angle2 = ceil(radToDeg(acos((pX^2 + pY^2 - (L1^2 + L2^2))/2*L1*L2)));
		int angle3 = ceil(radToDeg(ang - (0.5*asin((pX^2 + pY^2 - L1^2)/L2^2))));
		
		int answer[3] = {angle1, angle2, angle3};
		return answer;
	}
	
	//drive to the listed position inverse kinematics
	//drive(10,10,0,90)
	int drive(double x, double y, double z, double ang)
	{		
		int angleData[3] = angleCalc(x,y,z,ang);
			//check which gear it's on
			if(abs(posGear - 75) < 10) //arm
			{
				std::cout<< "Gear is driving main arm."<<std::endl;
				//11cm
					
				//open drive
				duty_cycle_sp.open(drive+"/duty_cycle_sp",std::ofstream::out);
				position_sp.open(drive+"/position_sp", std::ofstream::out);
				command.open(drive+"/command",std::ofstream::out);
				position.open(drive+"/position");
				
				position << ang1 << std::endl;
	
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << angleData[0] << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;
	
				Sleep(3);
				ang1 = angleData[0];
					
				//open gears
				duty_cycle_sp.open(gear+"/duty_cycle_sp",std::ofstream::out);
				position_sp.open(gear+"/position_sp", std::ofstream::out);
				command.open(gear+"/command",std::ofstream::out);
				position.open(gear+"/position");
				
	
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << 165 << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;
			}	
			else if(abs(posGear - 165) < 15 ) //forearm
			{ 
				std::cout<< "Gear is driving forearm."<<std::endl;
				//measure degrees the forearm moves when reached the tacho value: 7915
				//use ratio between them to figure out location of arm
				//20cm
					
				//open drive
				duty_cycle_sp.open(drive+"/duty_cycle_sp",std::ofstream::out);
				position_sp.open(drive+"/position_sp", std::ofstream::out);
				command.open(drive+"/command",std::ofstream::out);
				position.open(drive+"/position");
					
				position << ang2 << std::endl;

				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << angleData[1] << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;
		
				Sleep(3);
				ang2 = angleData[1];	

				//open gears
				duty_cycle_sp.open(gear+"/duty_cycle_sp",std::ofstream::out);
				position_sp.open(gear+"/position_sp", std::ofstream::out);
				command.open(gear+"/command",std::ofstream::out);
				position.open(gear+"/position");
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << 23 << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;					
			}	
			else if(abs(posGear-23) < 23) //finger
			{
				std::cout<< "Gear is driving finger."<<std::endl;
				//9.5cm
					
				//open drive
				duty_cycle_sp.open(drive+"/duty_cycle_sp",std::ofstream::out);
				position_sp.open(drive+"/position_sp", std::ofstream::out);
				command.open(drive+"/command",std::ofstream::out);
				position.open(drive+"/position");
				
				position << ang3 <<endl;
	
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << angleData[2] << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;
		
				Sleep(3);
				ang3 = angleData[2];
					
				//open gears
				duty_cycle_sp.open(gear+"/duty_cycle_sp",std::ofstream::out);
				position_sp.open(gear+"/position_sp", std::ofstream::out);
				command.open(gear+"/command",std::ofstream::out);
				position.open(gear+"/position");
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::endl;
				std::cout << "Set position." << std::endl;
				position_sp << 75 << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;
			}
			else
			{
				std::cout<< "Gear is still shifting, please wait."<<std::endl;
			}						
		}
	
	
