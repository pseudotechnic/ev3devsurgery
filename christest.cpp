#include <iostream>
#include <fstream>
#include <unistd.h>

	std::ofstream duty_cycle_sp;
	std::ofstream time_sp;
	std::ofstream position_sp;
	std::ofstream command;
	
	std::string roll ="/sys/class/tacho-motor/motor0";
	std::string drive = "/sys/class/tacho-motor/motor1";
	std::string gear = "/sys/class/tacho-motor/motor2";

int main(int argc, String* argv[]){
	
	int pos;
	position >> pos;
	std::cout << "Current Position: " << pos << std::endl;
	
	if(pos == 0)
	{
		std::cout << "Starting Operation..." << std::endl;
		std::cout << "Please enter motor name, duty cycle, time, and position."
		
		if(!argc < 4)
		{
			std::cout << "Usage is -in <infile> -out <outdir>\n";
			std::cin.get()
			exit(0);	
		}
		else
		{
			String* myMotor, myCommand, myDuty, myTime, myPosition;
			std::cout << argv[0];
			for(int i = 1; i < argc; i++)
			{
				if(i+1 != argc)
				{
					if(argv[i] == "-m")
					{
						myMotor = argv[i+1];
					}
					else if (argv[i] == "-c")
					{
						myCommand = argv[i+1];
					}
					else if (argv[i] == "-d")
					{
						myDuty = argv[i+1];
					}
					else if(argv[i] == "-t")
					{
						myTime = argv[i+1];
					}
					else if(argv[i] == "-p")
					{
						myPosition = argv[i+1];
					}
					else
					{
						std::cout << "Not enough or invalid arguments, please try again. \n";
						Sleep(2000);
						exit(0);
					}
				}
				std::cout << argv[i] << " ";
			}
			
			std::cin.get();
			return 0;
		}
	
		if(myMotor == "roll")
		{
			duty_cycle_sp.open(roll+"/duty_cycle_sp",std::ofstream::out);
			time_sp.open(roll+"/time_sp", std::ofstream::out);
			position_sp.open(roll+"/position_sp", std::ofstream::out);
			command.open(roll+"/command",std::ofstream::out);
			
			position.open(roll+"/position");
		}
		else if(myMotor == "gear")
		{
			duty_cycle_sp.open(drive+"/duty_cycle_sp",std::ofstream::out);
			time_sp.open(drive+"/time_sp", std::ofstream::out);
			position_sp.open(drive+"/position_sp", std::ofstream::out);
			command.open(drive+"/command",std::ofstream::out);
			
			position.open(gear+"/position");
		}
		else if(myMotor == "drive")
		{
			duty_cycle_sp.open(command+"/duty_cycle_sp",std::ofstream::out);
			time_sp.open(command+"/time_sp", std::ofstream::out);
			position_sp.open(command+"/position_sp", std::ofstream::out);
			command.open(command+"/command",std::ofstream::out);
			
			position.open(drive+"/position");
		}
		else
		{
			std::cout << "Invalid Motor Name, try again. \n" << std::endl;
		}
	
		std::cout <<"Set duty cycle." <<std::endl;
		duty_cycle_sp << myDuty << std::end1;
		
		std::cout << "Set time." << std::endl;
		time_sp << myTime << std::endl;

		std::cout << "Set command." << std::endl;
		command << myCommand << std::endl;
		
		std::cout << "Set position." << std::endl;
		command << myPosition << std::endl;
	
		Sleep(myTime);
	
	}		



	
	
	
	
}