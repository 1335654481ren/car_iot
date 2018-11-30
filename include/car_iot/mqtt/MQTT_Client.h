/*
/// @cond EXCLUDE
*/
#if !defined(MQTT_CLIENT_H)
#define MQTT_CLIENT_H

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>
#include <json/json.h>

#if defined(__cplusplus)
 extern "C" {
#endif
 	
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MQTTAsync.h"
#include "MQTTClient.h"
#if !defined(WIN32)
#include <unistd.h>
#else
#include <windows.h>
#endif

#if defined(_WRS_KERNEL)
#include <OsWrapper.h>
#endif

#define ADDRESS     "tcp://localhost:1883"
//#define ADDRESS      "tcp://ngrok.xiaomiqiu.cn:36508"
#define CLIENTID    "car_client"
#define TOPIC       "MQTT Examples"
#define TOPIC1      "MQTT Examples"
#define PAYLOAD     "Hello World!"
#define QOS0         0
#define QOS1         1
#define TIMEOUT     10000L

int init_mqtt_client(std::string clientID,std::string mqtt_url, ros::Publisher &ros_pub);
int mqtt_pub_message(char *topic, char *msg, int qos_level);
void disconnect_mqtt_client();
std::vector<std::string> get_reply_topic();

int get_flag();
void set_flag();

#ifdef __cplusplus
     }
#endif

#endif

