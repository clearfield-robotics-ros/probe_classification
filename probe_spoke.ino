#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

float motorPosVector_sim[] = { -.06, -.059, -0.055, -.048};

std_msgs::Float32 probe_data;
void probeCmdClbk(const std_msgs::Bool& msg);
ros::Publisher probe_data_pub("probe_data", &probe_data);
ros::Subscriber<std_msgs::Bool> probe_cmd_sub("begin_probe", probeCmdClbk);
int sampleTime_step = 0; 
void probeCmdClbk(const std_msgs::Bool& msg)
{
  if (msg.data) {
    nh.loginfo("Sending data...");
    probe_data.data = motorPosVector_sim[sampleTime_step];
    probe_data_pub.publish(&probe_data);
    sampleTime_step++;
  }
}

void setup()
{
  nh.initNode();
  nh.advertise(probe_data_pub);
  nh.subscribe(probe_cmd_sub);
}

void loop()
{
  nh.spinOnce();
}

// rosrun rosserial_python serial_node.py /dev/ttyACM0
// have to upload each time to reset the sampleTime_step variable to 0
