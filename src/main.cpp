#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <iostream>                                                 
#include <vector>
#include <pthread.h>
#include <unistd.h>
#include <queue>
#include <thread>         // std::thread
#include <mutex>          // std::mutex

#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include "otacli/ota_client.h"
#include "otacli/hobot_utils.h"
#include "otacli/ota_typedef.h"
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <rosgraph_msgs/Log.h>
#include "client_wss.hpp" 
#include "carmsg.pb.h"
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cstdio>
#include <opencv2/opencv.hpp>  
#include "tf/transform_datatypes.h"
#include <cv_bridge/cv_bridge.h>                                                
#include <sensor_msgs/image_encodings.h>                                        
#include <opencv2/opencv.hpp>                                                   
#include <nmea_msgs/Sentence.h>
#include <autodrive_msgs/Obstacles.h>
#include <autodrive_msgs/VehicleStatus.h>
//eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include "otacli/easywsclient.hpp"
#include <json/json.h>
#include "otacli/easyhttp_client.h"
#include <curl/curl.h>

#include <MQTT_Client.h>

using namespace std;
using namespace cv;
VideoCapture *capture = NULL;
queue<cv::Mat> list_image;
std::mutex mtx;
#define TEST_TAG "hobottest"
double count = 0;
int websocket_status = 0;
const char *tenant_token ="eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9."
"eyJleHAiOjE4NDE5MDM0OTAsImp0aSI6IjU2ZGQ3YWRiLWU5NjktNDhlZS04NjUyLTBmYzA1ZjBhNzAwNCIsImlzcyI6IkhvYm90Iiwic3ViIjoidGVzdDI"
"yQGhvYm90LmNjIiwic2NwIjoiaG9ib3QuKiIsImhvYm90LnRlbmFudCI6Im1pZGVhIn0.eQp2qBaoOEVjulEtxyvn1GTWBC_g-"
"_djxrLJRCdDgY9rgWMxOhLy9Wodme0hSXBOt9ObnKq8bGiTENumxsesrocHhtkBykbfJz7ko9c2KsK5ltBF3D6Ff4rU2WMdF46MzsIfmdw98PfZrGavMQAC"
"9TqfybsfKTtvDaEKtJkqg_5WCf97NHed9mYcsoq_6gUVy6nc7t3NCyenJ8f8sg-"
"cub6cOo4QOwKO2Fb4qDOyIszHwfFxSDmSKSLrgNB_Nl0VofjX6YkyV8IK8CsY26MJ93ZFTRsIIOZnqnBVbDl1J49Em-DgXaBimeq3eR25bNcKS7IQ5AV-"
"AW8JUX91bgu7gPnpuj6JGIyJ6Nk6Sk4S-w59wPUukvDFqRc67uDgyG7RnrQFMT9ihctYz22zZ56OPMbJhByqIr_gX7DP2CDEGUbzhIgNBQ-"
"kkRxn1GnvGzGEJVZR6EG0KOv6ougwhCFQPt0NhHEeoCti69nHtblIS5QoKMzFsvLjfCDMcnN7K0nY";

using WssClient = SimpleWeb::SocketClient<SimpleWeb::WSS>;

WssClient *client = NULL;
#define MEDEL_CUNT      3
long int model_msg_count[MEDEL_CUNT] = {0,0};
std::string clientID;
ros::Publisher station_pub;

double x=-500,y=-550,z = 0.0, w = 0.0, yaw=0,pitch = 0.0, roll = 0,latitude=31.278663397448387,longitude=121.197249409452;
float speed = 0.0;

void write_token(){

    // no need to parse response
  std::string auth_tokens;
  auth_tokens.assign(tenant_token);
  // sotre token to file system.
  std::string filePath = "./tenant_token.pems";
  std::ofstream ofile(filePath);
  if (ofile.is_open()) {
    ofile << auth_tokens;
  }
  ofile.close();

}

void pub_station(int station){
    std_msgs::String msg;    
    msg.data = std::to_string(station);  
    ROS_INFO("pub station = %s\n", msg.data);  
    station_pub.publish(msg);
}

void pub_station_str(std::string station){
    std_msgs::String msg;    
    msg.data = station; 
    ROS_INFO("pub station = %s\n", msg.data);  
    station_pub.publish(msg);
}

void odomCallback(nav_msgs::Odometry odom){
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;
    //printf(" %f,%f,%f,%f\n",odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Quaternion tfq(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Transform tftrans;
    tftrans.setRotation(tfq);
    tftrans.getBasis().getRPY(roll, pitch, yaw);

    model_msg_count[0]++;
    //printf("x = %f y = %f yaw =%f\n", x,y,yaw);
}

void gnssCallback(nmea_msgs::Sentence gnss)
{
    std::string  data = gnss.sentence;
    if (data.substr(0, 6) == "$GPGGA") {
        
        vector<std::string> vec = iot::HobotUtils::split(data, ",");
        // qxwz_rtcm_sendGGAWithGGAString("$GPGGA,000001,3112.518576,N,12127.901251,E,1,8,1,0,M,-32,M,3,0*4B\r\n");
        if(vec[2].length() < 4){
            ;//printf("no gpgga\n");
        }else{
            latitude = atof(vec[2].c_str()) / 100;
            longitude = atof(vec[4].c_str()) / 100;
            //printf("latitude = %.7f longitude = %.7f\n", latitude,longitude);
        }
    }
}

void ctrlc_message(int s) //ctrl+c消息捕获函数
{
    printf("caught signal %d\n",s);
    disconnect_mqtt_client();
    exit(1);
}

long int get_timestamp()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    //printf("second:%ld\n",tv.tv_sec);  //秒
    //printf("millisecond:%ld\n",tv.tv_sec*1000 + tv.tv_usec/1000);  //毫秒
    return tv.tv_sec*1000 + tv.tv_usec/1000;
}

std::string generate_pos_msg()
{
    Json::Value root;
    root["version"] = "V1";
    root["type"] = 1;
    root["timestamp"] = get_timestamp();
    Json::Value data;
    Json::Value local_location;
    local_location["x"] = x;
    local_location["y"] = y;
    local_location["z"] = z;
    local_location["w"] = w;
    local_location["yaw"] = yaw;
    local_location["pitch"] = pitch;
    local_location["roll"] = roll;   
    data["local_location"] = local_location;
    Json::Value gps_location;
    gps_location["latitude"] = latitude;
    gps_location["longitude"] = longitude;
    data["gps_location"] = gps_location;
    root["data"] = data;
    std::string send_out = root.toStyledString();
    return send_out;
}

std::string generate_state_msg()
{
    Json::Value root;
    root["version"] = "V1";
    root["type"] = 2;
    root["timestamp"] = get_timestamp();
    Json::Value data;
    data["car_state"] = 1;
    data["car_error"] = "car ok";
    data["car_stage"] = 1;
    data["car_speed"] = speed;
    data["odom"] = model_msg_count[0];model_msg_count[0] = 50;
    data["planner"] = model_msg_count[1];model_msg_count[1] = 10;
    data["perception"] = model_msg_count[2];model_msg_count[2] = 10;
    root["data"] = data;
    std::string send_out = root.toStyledString();
    return send_out;
}

std::string generate_cmdack_msg(std::string sequence)
{
    Json::Value root;
    root["version"] = "v1";
    root["type"] = 5;
    root["timestamp"] = get_timestamp();
    Json::Value data;
        data["error_code"] = 1;
        data["error_desc"] = "ok";
        data["sequence"] = sequence;
        data["route"].append(1);
        data["route"].append(3);
        data["route"].append(6);
        data["route"].append(8);
    root["data"] = data;
    std::string send_out = root.toStyledString();
    return send_out;
}

std::string generate_arrive_ack_msg()
{
    Json::Value root;
    root["version"] = "V1";
    root["type"] = 3;
    root["timestamp"] = get_timestamp();
    Json::Value data;
        data["station_num"] = 1;
        data["x"] = 1.234;
        data["y"] = 1.2334;
        data["z"] = 1.2334;
    root["data"] = data;
    std::string send_out = root.toStyledString();
    return send_out;
}

void roslogCallback( const rosgraph_msgs::Log::ConstPtr& msg ){
    Json::Value root;
    root["version"] = "V1";
    root["type"] = 4;
    root["timestamp"] = get_timestamp();
    Json::Value data;
    data["node"] = msg->name;
    switch (msg->level)
    {
        case rosgraph_msgs::Log::FATAL:
            data["log_level"] = "FATAL";
            break;
        case rosgraph_msgs::Log::ERROR:
            data["log_level"] = "ERROR";
            break;
        case rosgraph_msgs::Log::WARN:
            data["log_level"] = "WARN";
            break;
        case rosgraph_msgs::Log::DEBUG:
            data["log_level"] = "DEBUG";
            break;
        case rosgraph_msgs::Log::INFO:
            data["log_level"] = "INFO";
            break;
        default:
            break;
    }
    data["info"] = msg->msg;
    data["file"] = msg->file;
    data["function"] = msg->function;
    data["line"] = msg->line;
    root["data"] = data;
    if(msg->level == rosgraph_msgs::Log::FATAL || msg->level == rosgraph_msgs::Log::ERROR)
    {
        std::string send_out = root.toStyledString();
        std::string log_topic = "log/" + clientID;
        mqtt_pub_message(log_topic.c_str(),send_out.c_str(),QOS0);
    }
}

void timer_callback(const ros::TimerEvent&)
{
    std::string send_msg = generate_state_msg();
    std::string pub_status_topic = "status/" + clientID;
    mqtt_pub_message(pub_status_topic.c_str(),send_msg.c_str(),QOS0);
}

void objectsCallback(autodrive_msgs::Obstacles msg_objs){
        model_msg_count[2]++;                           
}
void trajCallback(const autodrive_msgs::PlanningTraj::ConstPtr msg){
        model_msg_count[1]++;
}
void vehicleCallback(const autodrive_msgs::VehicleStatus::ConstPtr msg){
    speed = msg->speed;
}

void testcall(const std_msgs::String::ConstPtr& msgg)  
{  
  ROS_INFO("I heard: [%s]", msgg->data.c_str());
  std::string msg = msgg->data.c_str();
  if( msg == "station"){
    printf("arrived !!!!!\n");
    std::string arrived = generate_arrive_ack_msg();
    std::string pub_arrived_topic = "arrived/" + clientID;
    mqtt_pub_message(pub_arrived_topic.c_str(),arrived.c_str(),QOS1);
  }else if( msg == "route"){
    std::vector<std::string > v  = get_reply_topic();

    std::string cmdack = generate_cmdack_msg(v[1]);

    std::string pub_cmdack_topic = "cmdack/" + clientID;
    mqtt_pub_message(pub_cmdack_topic.c_str(),cmdack.c_str(),QOS1);
  } 
}  

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_iot");
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh("~");

    std::string cfg_path,mqtt_url,baseUrl,imei,map_id;
    priv_nh.param("cfg", cfg_path, std::string(""));
    priv_nh.param("imei", imei, std::string(""));
    priv_nh.param("map_id", map_id, std::string(""));
    priv_nh.param("mqtt_url", mqtt_url, std::string("tcp://localhost:1883"));
    priv_nh.param("https_url", baseUrl, std::string("127.0.0.1:12345"));
    
    //ros::Subscriber log = nh_.subscribe("/rosout", 1, &(roslogCallback));

    ros::Subscriber gnss = nh_.subscribe("/nmea_sentence", 4, gnssCallback);
    ros::Subscriber novatel_odom = nh_.subscribe("/fusion/odom", 4, odomCallback);
    ros::Subscriber planner = nh_.subscribe("/planning/trajectory", 10, odomCallback);
    ros::Subscriber lidar_track_objects = nh_.subscribe("/track_pub_polygon_pc", 10, objectsCallback);
    
    ros::Subscriber  vehicle_status = nh_.subscribe("/vehicle/status", 10, vehicleCallback);

    station_pub = nh_.advertise<std_msgs::String>("/station", 1);

    ros::Subscriber  test = nh_.subscribe("/test", 1, testcall);

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrlc_message;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT,&sigIntHandler,NULL);

    std::string downloadPath = "/tmp";
    std::string storePath = cfg_path + "/";
    std::string token_file = cfg_path + "/tenant_token.pem";
    std::string tenant_token;
    std::ifstream file_stream(token_file);

    if(!file_stream.good())
    {
        std::cout << "not find ./tenant_token.pem" << std::endl;
        return 0;
    }else{
        std::stringstream buffer;
        std::ifstream in(token_file);
        buffer << in.rdbuf();
        tenant_token = buffer.str();
    }

    iot::OtaClient *iot_cli = new iot::OtaClient();

    iot_cli->Init(downloadPath,storePath,baseUrl,imei);

    char temp[100];
    sprintf(temp,"{\"imei\": \"%s\"}",imei.c_str());
    std::string id_data = temp;//"{\"imei\": \"ceb7ce8a-c82a-491c-9822-0847d4f71cdf\"}";

    std::string newTenantToken;
    int ret = iot_cli->AuthDevice(id_data, tenant_token, newTenantToken);
    if(ret == 0) {
        std::cout <<"AuthDevice Succeed!\n" << newTenantToken << std::endl;
        iot_cli->DownLoadMap(map_id);
    } else {
        std::cout <<"AuthDevice Failed!\n";    
        delete iot_cli;
        return 0;
    }

    clientID = iot_cli->get_device_id();
    if(clientID.empty()){
        printf("can not get device_id_!!!\n");
        delete iot_cli;
        return 0;
    }

    if(!init_mqtt_client(clientID, mqtt_url,station_pub)){
        printf("connect to mqtt server ok! \n");
    }

    ros::Timer timer = nh_.createTimer(ros::Duration(1), timer_callback);
    char msg[100];
    ros::Rate loop_rate(10);
    int count = 0;
    while(ros::ok()){
        count++;
        ros::spinOnce();

        std::string send_msg = generate_pos_msg();
        std::string pub_localt_topic = "location/" + clientID;
        mqtt_pub_message(pub_localt_topic.c_str(), send_msg.c_str(), QOS0);
        loop_rate.sleep();

        if(get_flag() == 1)
        {
          std::vector<std::string > v  = get_reply_topic();

          std::string cmdack = generate_cmdack_msg(v[1]);

          std::string pub_cmdack_topic = v[0];
          mqtt_pub_message(pub_cmdack_topic.c_str(),cmdack.c_str(),QOS1);

          printf("pub ack to : %s \n %s \n", pub_cmdack_topic.c_str(),cmdack.c_str());
          set_flag();
        }
    }
    disconnect_mqtt_client();
    return 0;
}













