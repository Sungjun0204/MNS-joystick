/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>


//  global variables
std::string mns_data;


serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

void mns_callback(const std_msgs::String::ConstPtr& msg) {
    mns_data = msg->data;
}






int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Subscriber write_sub2 = nh.subscribe("mns_msgs", 1000, mns_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(10);
    while(ros::ok()){

        ros::spinOnce();


        if(mns_data == "Comm_Start"){
            //// 통신 시작 ////
            ser.write("SYST:REM\n");   
            ser.write("OUTP 1\n");
            ser.write("MODE DC\n");s
            ser.write("INSTrument:COUPle NONE\n");
            ser.write("OUTP:IMM 1\n");
            // ser.write("VOLTage 10\n");
        }

        else if(mns_data == "Comm_End"){
            //// 통신 종료 ////
            ser.write("INSTrument:COUPle ALL\n");
            ser.write("VOLTage 0\n");
            ser.write("OUTP 0\n");
            ser.write("SYST:LOC\n");
            ser.write("OUTP:IMM 0\n");
        }
        
        //ROS_INFO_STREAM("SYST:REM");   // 문구 출력
        
    

        if(ser.available()){
            // ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}

