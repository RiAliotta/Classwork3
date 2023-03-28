#include "ros/ros.h"
#include "cw3/my_msg.h"
#include "std_msgs/Float32.h"
#include "boost/thread.hpp"
#define PI 3.1412

using namespace std;

class Filter_Class {
	public:
		Filter_Class(float dt);
		void topic_cb(cw3::my_msg::ConstPtr data); // Callback function reading data from msg
        void filter_fcn(); // Filters the data
	
	private:
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub;
        ros::Publisher _topic_pub;
        float _status;
        float _input;
        bool _ready;
        float _dt;
};

Filter_Class::Filter_Class(float dt) {

    _topic_sub = _nh.subscribe("/signal", 1, &Filter_Class::topic_cb, this);
    _topic_pub = _nh.advertise<std_msgs::Float32>("/filtered_signal", 1);
    _input = 0;
    _status = 0;
    _ready = false;
    _dt = dt;

    boost::thread(&Filter_Class::filter_fcn, this);
}

//Callback function: the input of the function is the data to read
//	In this function, a smart pointer is used
void Filter_Class::topic_cb(cw3::my_msg::ConstPtr data) {

	//data is a pointer of std_msgs::Int32 type
	//	to access to its field, the "." can not be used
	//ROS_INFO("Amplitude: %f \t Period: %f \t Current Value: %f", data->amplitude, data->period, data->value);
    //Filter_Class::filter_fcn(data->value);
    _ready = true;
    _input = data->value;

}

void Filter_Class::filter_fcn(){

    std_msgs::Float32 msg;
    while(ros::ok()){

        if(_ready){

            _ready = false;
            _status = (1-_dt/2)*_status + 5*_dt/2*_input;
            ROS_INFO("Filtered Value: %f", _status);
            msg.data = _status;
            _topic_pub.publish(msg);
            usleep(_dt*1e6);

        }
        

    }


}

int main(int argc, char** argv){

    float dt;

    if(argc > 1){

        dt = atof(argv[1]);

    }else{

        dt = 0.1;

    }

    ros::init(argc, argv,"filter");

    Filter_Class filter(dt);

    ros::spin();

}