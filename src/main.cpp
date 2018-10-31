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
//eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include "otacli/easywsclient.hpp"
#include <json/json.h>
#include "otacli/easyhttp_client.h"
#include <curl/curl.h>

using namespace std;
using namespace cv;
VideoCapture *capture = NULL;
queue<cv::Mat> list_image;
std::mutex mtx;
#define TEST_TAG "hobottest"
double count = 0;
int websocket_status = 0;
// const char *tenant_token ="eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9."
// "eyJleHAiOjE4NDE5MDM0OTAsImp0aSI6IjU2ZGQ3YWRiLWU5NjktNDhlZS04NjUyLTBmYzA1ZjBhNzAwNCIsImlzcyI6IkhvYm90Iiwic3ViIjoidGVzdDI"
// "yQGhvYm90LmNjIiwic2NwIjoiaG9ib3QuKiIsImhvYm90LnRlbmFudCI6Im1pZGVhIn0.eQp2qBaoOEVjulEtxyvn1GTWBC_g-"
// "_djxrLJRCdDgY9rgWMxOhLy9Wodme0hSXBOt9ObnKq8bGiTENumxsesrocHhtkBykbfJz7ko9c2KsK5ltBF3D6Ff4rU2WMdF46MzsIfmdw98PfZrGavMQAC"
// "9TqfybsfKTtvDaEKtJkqg_5WCf97NHed9mYcsoq_6gUVy6nc7t3NCyenJ8f8sg-"
// "cub6cOo4QOwKO2Fb4qDOyIszHwfFxSDmSKSLrgNB_Nl0VofjX6YkyV8IK8CsY26MJ93ZFTRsIIOZnqnBVbDl1J49Em-DgXaBimeq3eR25bNcKS7IQ5AV-"
// "AW8JUX91bgu7gPnpuj6JGIyJ6Nk6Sk4S-w59wPUukvDFqRc67uDgyG7RnrQFMT9ihctYz22zZ56OPMbJhByqIr_gX7DP2CDEGUbzhIgNBQ-"
// "kkRxn1GnvGzGEJVZR6EG0KOv6ougwhCFQPt0NhHEeoCti69nHtblIS5QoKMzFsvLjfCDMcnN7K0nY";

using WssClient = SimpleWeb::SocketClient<SimpleWeb::WSS>;

WssClient *client = NULL;

ros::Publisher station_pub; 

double x=-500,y=-550,yaw=0,latitude=31.278663397448387,longitude=121.197249409452;

void get_station();
void reset_station();

void wss_recv_todo(const char *msgs,int data_size)
{
    pbcar::MessageWrapper msg_proto;
    bool flag = msg_proto.ParseFromArray(msgs,data_size);
    if(msg_proto.has_server_cmd())
    {
        pbcar::ServerCmd *cmd_msg = msg_proto.mutable_server_cmd();
        int cmd_id = cmd_msg->cmd();
        long int timestamp = cmd_msg->timestamp();
        printf("recv : cmd_id = %d  timestamp = %ld\n", cmd_id, timestamp);
    }
}


void* Thread_wss_client_send(void*) {
  sleep(5);
  while(1){
    auto send_stream = make_shared<WssClient::SendStream>();
    
    pbcar::MessageWrapper send_msg;
    pbcar::ImageUpload *image_msg = send_msg.mutable_image_upload();
    image_msg->set_timestamp(1534921769);
    image_msg->set_session("AUTO_1234");
    image_msg->set_sequence("image");
    mtx.lock();
    //for(int j=0;j<5;j++)
    {
      cv::Mat image = list_image.front();
      double scale= 0.4; 
      Size dsize = Size(image.cols*scale,image.rows*scale);
      Mat dst = Mat(dsize,CV_32S);
      resize(image,dst,dsize);  
      imshow("Combine",dst);    
      waitKey(2);
      std::vector<uint8_t> dataVec;
      std::vector<int> param = std::vector<int>(2);
      param[0] = CV_IMWRITE_JPEG_QUALITY;
      param[1] = 95;
      cv::imencode(".jpg",dst,dataVec,param);
      std::string str_encode(dataVec.begin(), dataVec.end());
      char *data = (char *)str_encode.c_str();
      int data_size = str_encode.size();
      //printf("image size = %d\n",data_size);
      // image_msg->add_snapshot(data,data_size);
      // list_image.pop();
    }
    mtx.unlock();

    // std::string imagestr;
    // bool err =send_msg.SerializeToString(&imagestr);
    // if(!err)
    // {
    //     std::cout << "SerializeToString failed " << err;
    //     continue;
    // }

    // *send_stream << imagestr;
    // if(client != NULL)
    //   client->connection->send(send_stream); 

    // sleep(1);
    // std::string message = "hello world!";
    // cout << "Client: Sending message: \"" << message << "\"" << endl;
    // *send_stream << message;
    // if(client != NULL)
    //   client->connection->send(send_stream);        
  }
}

void* Thread_wss_client(void*) {

  //client = new WssClient("localhost:8181/echo", false);
  client = new WssClient("42.62.85.20:9448/ws/text", false);
  client->on_message = [](shared_ptr<WssClient::Connection> connection, shared_ptr<WssClient::Message> message) {
    auto message_str = message->string();

    cout << "Client: Message received: \"" << message_str << "\"" << endl;
    //deal server
    wss_recv_todo(message_str.c_str(),message_str.size());
  };

  client->on_open = [](shared_ptr<WssClient::Connection> connection) {
    cout << "Client: Opened connection" << endl;
  };

  client->on_close = [](shared_ptr<WssClient::Connection> /*connection*/, int status, const string & /*reason*/) {
    cout << "Client: Closed connection with status code " << status << endl;
  };

  // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
  client->on_error = [](shared_ptr<WssClient::Connection> /*connection*/, const SimpleWeb::error_code &ec) {
    cout << "Client: Error: " << ec << ", error message: " << ec.message() << endl;
  };

  client->start();

  return 0;
}

class myClassCallback: public iot::DownloadCallback {
  public:
    virtual void Process(void *p, double dTotal, double dLoaded) {
      std::cout << "dTotal size is " <<dTotal << ". Already download " <<dLoaded<< std::endl ; 
    }
};

void Sleep() {
  std::cout <<"*********Begin Sleep 30 Seconds**************\n";
  sleep(5);
  std::cout <<"***********End Sleep 30 Seconds**************\n";
}

void imageCallback(const sensor_msgs::ImageConstPtr& front_image)
{
    cv::Mat  image_raw = cv_bridge::toCvShare(front_image, "bgr8")->image;

    if(list_image.size() > 10)
      return;
    list_image.push(image_raw);//back  
}

int main_wss(int argc, char **argv)
{
  // no need to parse response
  // std::string auth_tokens;
  // auth_tokens.assign(tenant_token);
  // // sotre token to file system.
  // std::string filePath = "./tenant_token.pems";
  // std::ofstream ofile(filePath);
  // if (ofile.is_open()) {
  //   ofile << auth_tokens;
  // }
  // ofile.close();

  ros::init(argc, argv, "car_iot");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  std::string cfg_path;
  priv_nh.param("cfg", cfg_path, std::string(""));
  ros::Subscriber image = nh.subscribe("/sensor/matrix1/image1", 1, &(imageCallback));
  capture = new VideoCapture(0);

  // pthread_attr_t attr1;
  // pthread_attr_init(&attr1);
  // pthread_t th1;
  // pthread_create(&th1, &attr1, Thread_wss_client,0);

  // pthread_attr_t attr2;
  // pthread_attr_init(&attr2);
  // pthread_t th2;
  // pthread_create(&th2, &attr2, Thread_wss_client_send,0);


  std::string downloadPath = "/tmp";
  std::string storePath = cfg_path + "/";
  std::string token_file = cfg_path + "/tenant_token.pem";
  std::ifstream file_stream(token_file);
  if(!file_stream.good())
  {
      std::cout << "not find ./tenant_token.pem" << std::endl;
      return 0;
  }
  std::stringstream buffer;
  std::string tenent_file = cfg_path + "/tenant_token.pem";
  std::ifstream in(tenent_file);
  buffer << in.rdbuf();
  std::string tenant_token = buffer.str();

  iot::OtaClient *iot_cli = new iot::OtaClient();

  iot_cli->Init(downloadPath,storePath,"");
  std::string auth_token;
  std::string id_data = "{\"device_id\": \"test_id_02\"}";
  std::string newTenantToken;
  int ret = iot_cli->AuthDevice(id_data, tenant_token, newTenantToken);
  if(ret == 0) {
    std::cout <<"AuthDevice Succeed!\n" << newTenantToken << std::endl;
  } else {
    std::cout <<"AuthDevice Failed!\n";    
    delete iot_cli;
    return -1;
  }

  // std::string attrJson = 
  // "[{\"name\":\"artifact_name\",\"value\":\"001\"},{\"name\":\"device_type\",\"value\":\"otav1_type\"}]";
  // ret = iot_cli->ReportDeviceAttrs(attrJson);
  // if(ret == 0) {
  //   std::cout <<"ReportDeviceAttrs Succeed!\n";
  // } else {
  //   std::cout <<"ReportDeviceAttrs Failed!\n";
  // }
  std::string artifactName = "001";
  std::string artifactType = "otav1_type";
  std::string localPath    = "/home/rb-xu/dev/c++/otacli-dev/build/";
  while(true) {

    Mat frame;
    *capture >>frame;
      // imshow("Combine1",frame);    
      // waitKey(2);
    //iot_cli->QueryServerVersion();
    mtx.lock();
    if(list_image.size() > 10)
      list_image.pop();
    list_image.push(frame);
    mtx.unlock();    
  }
  while(true) {
    if (ret == static_cast<int>(iot::ErrorCode::kUnauthorized)) {
      iot_cli->AuthDevice(id_data, tenant_token, newTenantToken);
    }
    myClassCallback cb;
    ret = iot_cli->QueryUpdatorPackage(artifactName, artifactType);
    if (ret == 0) {
      ret = iot_cli->DownLoadUpdatorPackage(localPath, &cb, NULL);
    } else if(ret == static_cast<int>(iot::ErrorCode::kNoContent)) {
      std::cout << "No Update Package Detected.\n";
      Sleep();
      continue;
    } else {
      std::cout << "QueryUpdatorPackage Failed.\n";
      Sleep();
      continue;
    } 
#if 1 
    if(ret == 0) {
      std::string statusJson = "{\"status\":\"downloading\"}";
      ret = iot_cli->ReportDeployStatus(statusJson);
      if (ret == 0) {
        std::cout<<"ReportDeployStatus 'success' Succeed.\n";
      } else {
        std::cout<<"ReportDeployStatus 'success' Failed.\n";
      }
    } else {
      std::string statusJson = "{\"status\":\"downloading\"}";
      ret = iot_cli->ReportDeployStatus(statusJson);
      if (ret == 0) {
        std::cout<<"ReportDeployStatus 'failed' Succeed.\n";
      } else {
        std::cout<<"ReportDeployStatus 'failed' Failed.\n";
      }
    }
#endif
    std::string logJson = "{\"messages\":[{\"level\":\"debug\",\"message\":\"debug\",\"timestamp\":\"2012-12-23T22:08:41+00:00\"}]}";
    ret = iot_cli->ReportDeployLogs(logJson);
    if (ret == 0) {
      std::cout<< "ReportDeployLogs Succeed. \n";
    } else {
      std::cout<< "ReportDeployLogs Failed \n";
    }
    std::string fullAttrsJson;
    ret = iot_cli->QueryDeviceFullAttrs(fullAttrsJson);
    if(ret == 0) {
      std::cout<<"QueryDeviceFullAttrs succeed." << fullAttrsJson<<"\n";
    } else {
      std::cout<<"QueryDeviceFullAttrs Failed.\n";
    }
    std::string singleAttrsJson;
    ret = iot_cli->QueryDeviceAttrsByAttrName("device_type", singleAttrsJson);
    if(ret == 0) {
      std::cout<<"QueryDeviceAttrsByAttrName succeed." << singleAttrsJson<<"\n";
    } else {
      std::cout<<"QueryDeviceAttrsByAttrName Failed.\n";
    }
    Sleep();
  }
  if (iot_cli) {
		delete iot_cli;
		iot_cli = NULL;
	}

//  pthread_join(th1,NULL);
//  pthread_join(th2,NULL);
	std::cout<<"no test suit\n";
	return 0;
}


using easywsclient::WebSocket;
WebSocket *car_conn = NULL;

int flag = 0;


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

void handle_message(const std::string & message)
{
      Json::Reader reader;
      Json::Value config;
      printf("recve length =%s\n",message);
      if (reader.parse(message, config))
      {
          //读取根节点信息  
          std::string action = config["action"].asString();
          if( action == "cmd"){
              std::cout << "UseId:" << config["user_id"].asString();
              Json::Value root;
              root["action"] = "ack";
              root["name"] = "car";
              root["id"] = "EQ000001";
              root["status"] = "cmd action sucessesful";
              std::string out = root.toStyledString();
              int station_num = config["station"].asInt();
              pub_station(station_num);
              std::cout << "pub station ::" << station_num << std::endl;
              //car_conn->send(out);
          }
      }        
}


void update_status(std::string status){
      Json::Value update;
      update["action"] = "sync";
      update["name"] = "car";
      update["car_id"] = "EQ000001";
      update["type"] = "status";
      update["key"] = "car_status";
      update["value"] = status;
      std::string send_out1 = update.toStyledString();
      if( car_conn->getReadyState() == WebSocket::OPEN )
          car_conn->send(send_out1);
}

void register_car()
{
     Json::Value root;
      root["action"] = "register";
      root["name"] = "car";
      root["car_id"] = "EQ000001";
      root["user_id"] = "null";
      root["status"] = "avaliable";
      root["car_status"] = "init";
      root["sensor_status"] = "init";
      root["speed"] = 0.0;
      root["latitude"] = latitude;
      root["longitude"] = longitude;
      Json::Value arrayObj;
      Json::Value pos;
      pos["x"] = x;
      pos["y"] = y;
      pos["z"] = 0.0;
      root["pos"] = pos;
      root["yaw"] = yaw;
      root["pitch"] = 0.0;
      root["roll"] = 0.0;
      std::string out = root.toStyledString();
      if(car_conn->getReadyState() == WebSocket::OPEN)
          car_conn->send(out);
}


bool getUrl(std::string send_data)
{
    CURL *curl;
    CURLcode res;
    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Host: ihome.ngrok.xiaomiqiu.cn");
    headers = curl_slist_append(headers, "Connection: keep-alive");
    headers = curl_slist_append(headers, "Authorization: Basic dXNlcjoxMjM0NQ==");
    headers = curl_slist_append(headers, "Accept: application/json");
    curl = curl_easy_init();    // 初始化
    if (curl)
    {
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L);//请求超时时长
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L); //连接超时时长 
        string str_url= "http://ihome.ngrok.xiaomiqiu.cn/asp/car_pos.asp?pos=" + send_data;
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);// 改协议头
        curl_easy_setopt(curl, CURLOPT_URL,str_url.c_str());
        printf("send: %s\n", str_url.c_str());
        res = curl_easy_perform(curl);   // 执行

        long res_code=0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &res_code);
        printf("aa = %d  code = %d \n", res,res_code);
        if (res != 0) {

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }
        return true;
    }
}
size_t write_callback(void *ptr, size_t size, size_t nmemb, void *stream) {  
    int len = size * nmemb;  
    int written = len;   
    printf("recv len-----------:%d\n data = %s\n",written,(char *)ptr);

    Json::Value config;
    Json::Reader reader;
    std::string json_str = (char*)ptr;
    if (reader.parse(json_str, config))
    {
        if(config["status"].asString() == "success"){
            printf(" status = %s  station =%s\n",config["status"].asString().c_str(), config["station"].asString().c_str());
            if(config["station"].asString() != "0" && config["station"].asString() != ""){
              pub_station_str(config["station"].asString());
              reset_station();
            }

        }
    }
    return written;  
}  

int http_loced_request(std::string send_data)
{
    CURL *curl;
    CURLcode res;
    char cmd_buf[500];
    sprintf(cmd_buf,"pos=%s&end=end\0",send_data.c_str());
    int return_code = -1;
    /* In windows, this will init the winsock stuff */ 
    curl_global_init(CURL_GLOBAL_ALL);
    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Host: ihome.ngrok.xiaomiqiu.cn");
    headers = curl_slist_append(headers, "Connection: keep-alive");
    headers = curl_slist_append(headers, "Authorization: Basic dXNlcjoxMjM0NQ==");
    headers = curl_slist_append(headers, "Accept: application/json");
    /* get a curl handle */ 
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3L);//请求超时时长
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L); //连接超时时长 
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);// 改协议头

        /* First set the URL that is about to receive our POST. This URL can
           just as well be a https:// URL if that is what should receive the
           data. */ 
        //curl_easy_setopt(curl, CURLOPT_URL, "http://27.115.33.234:8081/attc/httpservices/phone/locked/ies");
        curl_easy_setopt(curl, CURLOPT_URL, "http://ihome.ngrok.xiaomiqiu.cn/asp/car_pos.asp");
        /* Now specify the POST data */ 
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, cmd_buf);
        //指定回调函数  
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);  
        //这个变量可作为接收或传递数据的作用  
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&return_code); 
        /* Perform the request, res will get the return code */ 
        res = curl_easy_perform(curl);
        /* Check for errors */ 
        if(res != CURLE_OK){
            fprintf(stderr, "curl_easy_perform() failed: %s\n",curl_easy_strerror(res));
            /* always cleanup */ 
            curl_easy_cleanup(curl);
            curl_global_cleanup();
            return -1;      
        }
        /* always cleanup */ 
        curl_easy_cleanup(curl);
    }
    //printf("return_code = %d\n",return_code);
    curl_global_cleanup();
    return return_code;
}

void update_pos(){
    static double sdep = 0;
    Json::Value root;
    root["action"] = "sync";
    root["name"] = "car";
    root["car_id"] = "EQ000001";
    root["type"] = "pos";
    root["latitude"] = latitude;
    root["longitude"] = longitude;
    Json::Value arrayObj;
    Json::Value pos;
    pos["x"] = x;
    pos["y"] = y;
    pos["z"] = 0.0;
    root["pos"] = pos;
    sdep+=0.1;
    root["speed"] = 0.0 + sdep;
    root["yaw"] = yaw;
    root["pitch"] = 0.0;
    root["roll"] = 0.0;
    std::string send_out = root.toStyledString();
    http_loced_request(send_out);
    // if( car_conn->getReadyState() == WebSocket::OPEN )
    //     car_conn->send(send_out);
    // else{
    //   websocket_status = 0;
    //   printf("reconnected to server\n");
    //   websocket_status = 1;
    //   http_loced_request(send_out);
    // }
}

void get_station(){
    static double sdep = 0;
    Json::Value root;
    root["action"] = "sync";
    root["name"] = "car";
    root["car_id"] = "EQ000001";
    root["type"] = "get_station";

    std::string send_out = root.toStyledString();
    http_loced_request(send_out);

}

void reset_station(){
    static double sdep = 0;
    Json::Value root;
    root["action"] = "sync";
    root["name"] = "car";
    root["car_id"] = "EQ000001";
    root["type"] = "reset_station";
    root["key"] = "sensor_status";
    root["value"] = "0";
    std::string send_out = root.toStyledString();
    http_loced_request(send_out);

}

void odomCallback(nav_msgs::Odometry odom){
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    //printf(" %f,%f,%f,%f\n",odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Quaternion tfq(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Transform tftrans;
    tftrans.setRotation(tfq);
    double roll, pitch, yaw1;
    tftrans.getBasis().getRPY(roll, pitch, yaw1);
    yaw = yaw1;
    //printf("x = %f y = %f yaw =%f\n", x,y,yaw);
}

std::vector<std::string> split(const  std::string& s, const std::string& delim)
{
    std::vector<std::string> elems;
    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();
    if (delim_len == 0) return elems;
    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }
    return elems;
}

void gnssCallback(nmea_msgs::Sentence gnss)
{
    std::string  data = gnss.sentence;
    //printf("recv: %s", data.c_str());
    if (data.substr(0, 6) == "$GPGGA") {
        
        vector<std::string> vec = split(data, ",");
        // qxwz_rtcm_sendGGAWithGGAString("$GPGGA,000001,3112.518576,N,12127.901251,E,1,8,1,0,M,-32,M,3,0*4B\r\n");
        if(vec[2].length() < 4){
            printf("no gpgga\n");
        }else{
            latitude = atof(vec[2].c_str()) / 100;
            longitude = atof(vec[4].c_str()) / 100;
            //printf("latitude = %.7f longitude = %.7f\n", latitude,longitude);
        }

    }
}

static void *pthread_service(void *ptr) {
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        //printf("recv msg............\n");
        //if(websocket_status == 1){
        //    car_conn->poll();
        //    car_conn->dispatch(handle_message);
        //}
        get_station();
        loop_rate.sleep();
    }
}

void ctrlc_message(int s) //ctrl+c消息捕获函数
{
    printf("caught signal %d\n",s);
    exit(1);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "manager_server");
    ros::NodeHandle nh_;
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrlc_message;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT,&sigIntHandler,NULL);

    //car_conn = WebSocket::from_url("ws://tcp.ngrok.xiaomiqiu.cn:36507");
    //car_conn = WebSocket::from_url("ws://localhost:8181");
    //assert(car_conn);
    
    //register_car();

    //update_status("online");
    websocket_status = 1;

    ros::Subscriber gnss = nh_.subscribe("/nmea_sentence", 4, gnssCallback);
    ros::Subscriber novatel_odom = nh_.subscribe("/fusion/odom", 4, odomCallback);
    station_pub = nh_.advertise<std_msgs::String>("/station", 1);

    pthread_t pid1;
    int ret1 = pthread_create(&pid1,NULL,pthread_service,NULL);
    if(ret1 < 0) {
        std::cout << "thread create fail" << std::endl;
    }
    ros::Rate loop_rate(1);

    while(ros::ok()){
        //printf("running ------\n");
        ros::spinOnce();
        update_pos();
        //get_station();
        loop_rate.sleep();
    }
    pthread_join(pid1,NULL);
    //delete car_conn;

    return 0;
}
