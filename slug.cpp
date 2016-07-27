#include "ev3dev.h"

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace ev3dev;

class control
{
	public:
		control();
		~control();
		
		void drive(int speed, int position);
		void stop();
		void reset();
		
	protected:
		large_motor     roll;
		large_motor     drive;
		large_motor		gear; 
	
	control::control() :
		roll(OUTPUT_A),
		drive(OUTPUT_B),
		gear(OUTPUT_C)
}

	std::ofstream duty_cycle_sp;
	std::ofstream time_sp;
	std::ofstream position_sp;
	std::ofstream command;
	
	std::string roll ="/sys/class/tacho-motor/motor0";
	std::string drive = "/sys/class/tacho-motor/motor1";
	std::string gear = "/sys/class/tacho-motor/motor2";
	
	int posRoll;
	int posGear;
	int posDrive;
	
	int L1 = 11;
	int L2 = 20;
	int L3 = 9.5;
	
	int ang1 = 0;
	int ang2 = 0;
	int ang3 = 0;
	
	int main()
	{
		drive(10,10,0,90);
	}
/*
	//get current position forward kinematics 
	int[] positionTracking()
	{
		position.open(roll+"/position") >> posRoll;
		position.open(gear+"/position") >> posGear;
		position.open(drive+"/position") >> posDrive;
		
	//check which gear it's on
		if(abs(posGear - 75) < 10)
		{
			isArm = True;
			std::cout<< "Gear is driving main arm."<<std::endl;
			if(currentAngle1 == wantedAngle1)
			{
				posDrive << 0 << std::end1;
			}
			//11cm
		}	
		else if(abs(posGear - 165) < 15 )
		{ 
			isFor = True;
			std::cout<< "Gear is driving forearm."<<std::endl;
			if(currentAngle1 == wantedAngle1)
			{
				posDrive << 0 << std::end1;
			}
			//measure degrees the forearm moves when reached the tacho value: 7915
			//use ratio between them to figure out location of arm
			//20cm
		}	
		else if(abs(posGear-23) < 23)
		{
			isFinger = True;
			std::cout<< "Gear is driving finger."<<std::endl;
			if(currentAngle1 == wantedAngle1)
			{
				posDrive << 0 << std::end1;
			}
			//9.5cm
			else
			{
				std::cout<< "Gear is still shifting, please wait."<<std::endl;
			}	
		//look at positionTracking.cpp 
		}
	}	*/
	//radian to degree converter
	double radToDeg(double x)
	{
		double answer = tan(x * PI / 180);
		return answer; 
	}
	
	//calculate the angles
	double[] angleCalc(double x, double y, double z, double ang)
	{		
		pXF = pX + L3*ang; 
		pXY = pY + L3*ang; 
		
		pX = L1*cos(angle1) + L2*cos(angle1 + angle2);
		pY = L2*sin(angle1) + L2*sin(angle1 + angle2); 
		
		double angle1 = radToDeg((0.5*asin((pX^2 + pY^2 - L1^2)/L2^2))-acos((pX^2 + pY^2 - (L1^2 + L2^2))/2*L1*L2));
		double angle2 = radToDeg(acos((pX^2 + pY^2 - (L1^2 + L2^2))/2*L1*L2));
		double angle3 = radToDeg(ang - (0.5*asin((pX^2 + pY^2 - L1^2)/L2^2)));
		
		double answer[3] = {angle1, angle2, angle3};
		return answer;
	}
	
	/*int driveTest(double x, double y, double z, double ang)
	{
		double angleData[3] = angleCalc(x,y,z,ang);
		
	}*/
	
	//drive to the listed position inverse kinematics
	//drive(10,10,0,90)
	int drive(double x, double y, double z, double ang)
	{		
		double angleData[3] = angleCalc(x,y,z,ang);
			/*//open roll 
			duty_cycle_sp.open(roll+"/duty_cycle_sp",std::ofstream::out);
			time_sp.open(roll+"/time_sp", std::ofstream::out);
			position_sp.open(roll+"/position_sp", std::ofstream::out);
			command.open(roll+"/command",std::ofstream::out);
			position.open(roll+"/position"); */
		
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
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::end1;
				std::cout << "Set position." << std::endl;
				command << angleData[0] << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;
	
				Sleep(3);
				ang1 = angleData[0];
					
				//open gears
				duty_cycle_sp.open(gear+"/duty_cycle_sp",std::ofstream::out);
				time_sp.open(gear+"/time_sp", std::ofstream::out);
				position_sp.open(gear+"/position_sp", std::ofstream::out);
				command.open(gear+"/command",std::ofstream::out);
				position.open(gear+"/position");
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::end1;
				std::cout << "Set position." << std::endl;
				command << 165 << std::endl;
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
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::end1;
				std::cout << "Set position." << std::endl;
				command << angleData[1] << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;
		
				Sleep(3);
				ang2 = angleData[1];	

				//open gears
				duty_cycle_sp.open(gear+"/duty_cycle_sp",std::ofstream::out);
				time_sp.open(gear+"/time_sp", std::ofstream::out);
				position_sp.open(gear+"/position_sp", std::ofstream::out);
				command.open(gear+"/command",std::ofstream::out);
				position.open(gear+"/position");
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::end1;
				std::cout << "Set position." << std::endl;
				command << 23 << std::endl;
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
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::end1;
				std::cout << "Set position." << std::endl;
				command << angleData[2] << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;
		
				Sleep(3);
				ang3 = angleData[2];
					
				//open gears
				duty_cycle_sp.open(gear+"/duty_cycle_sp",std::ofstream::out);
				time_sp.open(gear+"/time_sp", std::ofstream::out);
				position_sp.open(gear+"/position_sp", std::ofstream::out);
				command.open(gear+"/command",std::ofstream::out);
				position.open(gear+"/position");
					
				std::cout <<"Set duty cycle." <<std::endl;
				duty_cycle_sp << 100 << std::end1;
				std::cout << "Set position." << std::endl;
				command << 75 << std::endl;
				std::cout << "Set command." << std::endl;
				command << run_to_abs_pos << std::endl;
			}
			else
			{
				std::cout<< "Gear is still shifting, please wait."<<std::endl;
			}						
		}
	
	