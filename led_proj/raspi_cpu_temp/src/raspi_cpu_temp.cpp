#include <iostream>
#include <string>
#include <cstdio>
#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace std;

class CPU_temp
{
public:
	CPU_temp();
	~CPU_temp(){};
	void run();
private:
	string topic_name;
	FILE *thermal;
private:
	ros::NodeHandle nh;
	ros::Publisher cpuTempPub;
	std_msgs::Float32 cpuTemp;
};
CPU_temp::CPU_temp() : nh("~")
{
	topic_name = "/cpu_temp";
	cpuTempPub = nh.advertise<std_msgs::Float32>(topic_name, 10);
}

void CPU_temp::run()
{
	float millideg, temp;
	int n;

	ros::AsyncSpinner spinner(0);
	ros::Rate rate(30);
	spinner.start();
	while (ros::ok())
	{
		thermal = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
		n = fscanf(thermal, "%f", &millideg);
		temp = millideg / 1000;
		fclose(thermal);
		
		cpuTemp.data = temp;
		cpuTempPub.publish(cpuTemp);
		rate.sleep();
	}
	spinner.stop();	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cpu_temp_pub");
	CPU_temp cpu_temp;
	cpu_temp.run();

	return 0;
}
