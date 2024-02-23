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
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/Empty.h>
#include <string>
#include <vector>


//  global variables
std::string mns_data;
std::vector<float> voltage;
std::string vh, vuy, vuz, vm, vg;


serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

void mns_callback(const std_msgs::String::ConstPtr& msg) {
    mns_data = msg->data;
}

void voltarray_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    voltage.clear();
    for(const float& value : msg->data)
        voltage.push_back(value);
}




int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    // 메세지 구독 설정
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Subscriber write_sub2 = nh.subscribe("mns_msgs", 1000, mns_callback);    // 제어 모드 관련 커맨드 메세지 구독
    ros::Subscriber write_sub3 = nh.subscribe("volt_array", 1000, voltarray_Callback);  // helical mode 전압 값 메세지 구독


    // 메세지 발행 설정
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    ///////////////////////
    //// USART 통신 설정 ////
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

    /////////////////////////


    //// mode에 따른 통신 알고리즘 설정 ////

    int flag_on = 0;      // 통신 on 설정이 한 번만 눌리게끔 하는 플래그변수 설정
    int flag_off = 0;     // 통신 off 설정이 한 번만 눌리게끔 하는 플래그변수 설정

    ros::Rate loop_rate(100000);
    while(ros::ok()){

        ros::spinOnce();

        if(mns_data == "Comm_Start" && flag_on == 0){
            //// 통신 시작 ////
            ROS_INFO_STREAM("Comm On");      // 통신 시작 텍스트 출력
            ser.write("SYST:REM\n");   
            ser.write("OUTP 1\n");
            ser.write("MODE DC\n");
            ser.write("INSTrument:COUPle NONE\n");
            flag_on = 1; flag_off = 0;
        }

        else if(mns_data == "Comm_End" && flag_off == 0){
            //// 통신 종료 ////
            ROS_INFO_STREAM("Comm Off");     // 통신 종료 텍스트 출력
            ser.write("INSTrument:COUPle ALL\n");
            ser.write("OUTP 0\n");
            ser.write("VOLTage 0\n");
            ser.write("SYST:LOC\n");
            flag_on = 0; flag_off = 1;
        }


        else if(mns_data == "Control_Start" && flag_off == 0){
            // voltage 변수에 저장되어 있는 각 코일의 전압 값을 문자열로 변환 저장
            std::stringstream ss;
            if (voltage.size() >= 5) {
                for(int i = 0; i < 5; ++i) {
                    ss.str(""); // 스트림 초기화
                    ss << std::fixed << std::setprecision(3) << voltage[i];
                    switch(i) {
                        case 0: vh  = ss.str(); break;
                        case 1: vuy = ss.str(); break;
                        case 2: vuz = ss.str(); break;
                        case 3: vm  = ss.str(); break;
                        case 4: vg  = ss.str(); break;
                    }
                }
            }       
            
            ROS_INFO_STREAM("*** Helical Mode ***");     // 제어 연산 결과 값을 송신 중임을 알리는 텍스트 출력
            ser.write("INSTrument:NSELect 1\n");
            ser.write("VOLTage "); ser.write(vh); ser.write("\n");

            ser.write("INSTrument:NSELect 2\n");
            ser.write("VOLTage "); ser.write(vuy); ser.write("\n");

            ser.write("INSTrument:NSELect 3\n");
            ser.write("VOLTage "); ser.write(vuz); ser.write("\n");

        }
        
    

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

