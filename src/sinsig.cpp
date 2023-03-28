#include "ros/ros.h"
#include "cw3/my_msg.h"
#define PI 3.1412

int main(int argc, char** argv){

    float amplitude;
    float period;
    float value;

	if(argc > 2){

        amplitude = atof(argv[1]);
        period = atof(argv[2]);

    }else{

        amplitude = 1;
        period = 1;

    }
        
    value = 0;
    
    int system_freq = 10;
    
    ros::init(argc, argv,"sinsig");
	
	ros::NodeHandle nh;

	ros::Publisher sig = nh.advertise<cw3::my_msg>("/signal", 1);

	ros::Rate rate(system_freq); 

    cw3::my_msg data;
    
    data.amplitude = amplitude;
    data.period = period;

    int time = 0;
    float timef = 0.0;

	// Typical loop: neverending loop: a controller works until actuators are activated
	//		while (ros::ok()): works until the ROS node is not terminated (by the user with ctrl+c or similar)
	while ( ros::ok() ) {

        timef = (float) time;

		data.value = sin(timef/system_freq/period*2*PI);

		//ROS_INFO: Like a printf, but with the timestamp
		ROS_INFO("%f",data.value); 

		//Publish the message over the ROS network
		sig.publish(data);
		
        time = (time + 1) % (int) (system_freq*period);

		//Rate to maintain the 10 Hz
		rate.sleep();
	}
	
	return 0;

}