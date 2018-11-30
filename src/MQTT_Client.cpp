/*******************************************************************************
 * Copyright (c) 2012, 2018 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution. 
 *
 * The Eclipse Public License is available at 
 *   http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at 
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Ian Craggs - initial contribution
 *******************************************************************************/

#include <MQTT_Client.h>

volatile MQTTClient_deliveryToken deliveredtoken;

MQTTClient mqtt_client;
MQTTClient_connectOptions conn_opts;

std::string sub_command_topic;

std::string gclientID;

ros::Publisher ros_station_pub;

std::string reply_topic;
std::string sequence;

int flag = 0;

int get_flag()
{
    return flag;
}
void set_flag(){
    flag = 0;
}
void delivered(void *context, MQTTClient_deliveryToken dt)
{
    printf("Message with token value %d delivery confirmed\n", dt);
    deliveredtoken = dt;
}

std::vector<std::string> get_reply_topic()
{
    std::vector<std::string> vecd;
    vecd.push_back(reply_topic);
    vecd.push_back(sequence);
    return vecd;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    int i;
    char* payloadptr;

    printf("Message arrived\n");
    printf("     topic: %s\n", topicName);
    printf("   message: ");

    payloadptr = message->payload;
    Json::Reader reader;
    Json::Value config;
    printf("recve length =%s\n",payloadptr);
    if (reader.parse(payloadptr, config))
    {
      //读取根节点信息  
      int  type = config["type"].asInt();
      Json::Value data = config["data"];

      if( type != 4 ){
        Json::Value station = data["station"];
        reply_topic = data["reply_topic"].asString();
        sequence = data["sequence"].asString();
        int station_num = station["station"].asInt();

        std_msgs::String msg;    
        msg.data = std::to_string(station_num);  
        ROS_INFO("pub station = %s\n", msg.data);  
        ros_station_pub.publish(msg);
        flag = 1;
        std::cout << "pub station ::" << station_num << std::endl;
      }

    } 

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void connlost(void *context, char *cause)
{
    MQTTClient client = (MQTTClient)context;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;

    printf("\nConnection lost\n");
    printf("     cause: %s\n", cause);

    printf("Reconnecting\n");
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    while ((rc = MQTTClient_connect(mqtt_client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        printf("reconnect to server!\n");
        sleep(1);
    }
    sub_command_topic = "command/" + gclientID;
    printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n"
           "Press Q<Enter> to quit\n\n", sub_command_topic.c_str(), gclientID.c_str(), QOS1);
    
    MQTTClient_subscribe(mqtt_client, sub_command_topic.c_str(), QOS1);
}

int init_mqtt_client(std::string clientID,std::string mqtt_url, ros::Publisher &ros_pub)
{
    conn_opts = MQTTClient_connectOptions_initializer;
    int rc;
    int ch;
    gclientID = clientID;
    ros_station_pub = ros_pub;
    //MQTTClient_create(&mqtt_client, ADDRESS, CLIENTID,MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_create(&mqtt_client, mqtt_url.c_str(), gclientID.c_str(),MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    MQTTClient_setCallbacks(mqtt_client, NULL, connlost, msgarrvd, delivered);

    if ((rc = MQTTClient_connect(mqtt_client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        return -1;
    }
    sub_command_topic = "command/" + gclientID;
    printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n"
           "Press Q<Enter> to quit\n\n", sub_command_topic.c_str(), gclientID.c_str(), QOS1);
    
    MQTTClient_subscribe(mqtt_client, sub_command_topic.c_str(), QOS1);

    return 0;
}

void disconnect_mqtt_client()
{
    MQTTClient_unsubscribe(mqtt_client, sub_command_topic.c_str());
    MQTTClient_disconnect(mqtt_client, 10000);
    MQTTClient_destroy(&mqtt_client);
}

int mqtt_pub_message(char *topic, char *msg, int qos_level)
{
	MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int rc;
    pubmsg.payload = msg;
    pubmsg.payloadlen = (int)strlen(msg);
    pubmsg.qos = qos_level;
    pubmsg.retained = 0;
    MQTTClient_publishMessage(mqtt_client, topic, &pubmsg, &token);
    //printf("Waiting for up to %d seconds for publication of %s\n"
    //"on topic %s for client with ClientID: %s\n",
    //       (int)(TIMEOUT/1000), msg, topic, gclientID.c_str());
    rc = MQTTClient_waitForCompletion(mqtt_client, token, 100);
    //printf("Message with delivery token %d delivered\n", token);
    return rc;
}