#include <ros/ros.h> 
#include <robot_audio/robot_iat.h>
#include <robot_audio/Collect.h>
#include <robot_audio/robot_tts.h>
#include <actionlib/client/simple_action_client.h>
#include "move_base_msgs/MoveBaseAction.h"
#include <iostream>
#include <string>
#include <face_rec/recognition_results.h>
#include <relative_move/SetRelativeMove.h>
#include <ar_pose/Track.h>
#include <unistd.h> // 用于 sleep
using namespace std;

struct Point {
    float x;  // x坐标
    float y;  // y坐标
    float z;  // 姿态z
    float w;  // 姿态w
    string name; // 语音识别名字
    string name1; // 地点播报名字
    string present; // 介绍语
};

struct Point m_point[10] = {
    {1.020, 2.132,-0.707, 0.707 , "上海", "上上海馆", " 上上海是中国的经济中心"},
    {1.050, 1.171, -0.707, 0.707, "深圳", "深深圳馆", " 深深圳是中国的科创中心"},
    {2.523, 2.132, -0.707, 0.707, "吉林", "吉吉林馆", " 吉吉林位于我国东北是人参之都"},
    {2.521, 1.126, -0.707, 0.707, "广州", "广广州馆", " 广广州自古都是我国的商业之都"},
    {2.558, 0.079, -0.707, 0.707, "北京", "北北京馆", " 北北京是中国的首都，政治中心"},
 {0.000, -0.059, -1.000, 1.000, "起点", "起点", " 起始点"},
    {0.000, -0.059, 0.000, 1.000, "起点", "起点", " 起始点"},
    
};
struct Point m_goto[10] = {
    {1.020, 2.132, 1.000, 1, "上海", "上上海馆", " 上上海是中国的经济中心"},
    {1.050, 1.171, 1.000, 1, "深圳", "深深圳馆", " 深深圳是中国的科创中心"},
    {2.523, 2.132, 1.000, 1, "吉林", "吉吉林馆", " 吉吉林位于我国东北是人参之都"},
    {2.521, 1.126, 1.000, 1, "广州", "广广州馆", " 广广州自古都是我国的商业之都"},
    {2.558, 0.079, 1.000, 1, "北京", "北北京馆", " 北北京是中国的首都，政治中心"},
    {0.000, 0.059, 0.000, 1.000, "起点", "起点", " 起始点"},
    {0.400, 2.000, -0.707, 0.707, "充电桩", "充电桩", "冲充电成功"},
    {0.020, -0.049, -1.000, 1.000, "起点", "起点", " 起始点"},
    {0.020, -0.049, 0.000, 1.000, "起点", "起点", " 起始点"}
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> AC;

class Interaction {
public:
    Interaction();
    string voice_collect(); // 语音采集
    string voice_dictation(const char* filename); // 语音听写
    string voice_tts(const char* text); // 语音合成
    bool goto_nav(struct Point* point); // 导航到目标位置，返回是否成功
private:
    ros::NodeHandle n;
    ros::ServiceClient collect_client, dictation_client, tts_client;
};

Interaction::Interaction() {
    collect_client = n.serviceClient<robot_audio::Collect>("voice_collect");
    dictation_client = n.serviceClient<robot_audio::robot_iat>("voice_iat");
    tts_client = n.serviceClient<robot_audio::robot_tts>("voice_tts");
}

string Interaction::voice_collect() {
    ros::service::waitForService("voice_collect");
    robot_audio::Collect srv;
    srv.request.collect_flag = 1;
    collect_client.call(srv);
    return srv.response.voice_filename;
}

string Interaction::voice_dictation(const char* filename) {
    ros::service::waitForService("voice_iat");
    robot_audio::robot_iat srv;
    srv.request.audiopath = filename;
    dictation_client.call(srv);
    return srv.response.text;
}

string Interaction::voice_tts(const char* text) {
    ros::service::waitForService("voice_tts");
    robot_audio::robot_tts srv;
    srv.request.text = text;
    tts_client.call(srv);
    string cmd = "play " + srv.response.audiopath;
    system(cmd.c_str());
    sleep(1);
    return srv.response.audiopath;
}

bool Interaction::goto_nav(struct Point* point) {
    AC* ac = new AC("move_base", true);
    ROS_INFO("Waiting for action server to start.");
    ac->waitForServer();
    ROS_INFO("Action server started, sending goal.");
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = point->x;
    goal.target_pose.pose.position.y = point->y;
    goal.target_pose.pose.orientation.z = point->z;
    goal.target_pose.pose.orientation.w = point->w;
    ac->sendGoal(goal);
    ac->waitForResult();
    bool success = (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (success)
        ROS_INFO("Goal succeeded!");
    else
        ROS_INFO("Goal failed!");
    ac->cancelGoal();
    delete ac;
    return success;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "interaction");
    Interaction audio;
    string dir, text;
    
    while (ros::ok()) {
        dir = audio.voice_collect();
        text = audio.voice_dictation(dir.c_str()).c_str();
        if (text.find("元宝元宝") != string::npos) {
            audio.voice_tts("  哎，什么事呀");
            dir = audio.voice_collect();
            text = audio.voice_dictation(dir.c_str()).c_str();
          if (text.find("带我参观一圈") != string::npos) {
    audio.voice_tts("  好的，这就带您参观");
    for (int i = 0; i < 7; i++) {
        audio.goto_nav(&m_point[i]);
        audio.voice_tts(m_point[i].present.c_str());
        
        // 新增向后移动代码（保持安全距离）
        ros::NodeHandle nh;
        ros::ServiceClient move_client = nh.serviceClient<relative_move::SetRelativeMove>("relative_move");
        relative_move::SetRelativeMove move_srv;
        move_srv.request.goal.x = -0.5;  // 向后移动0.5米
        move_srv.request.global_frame = "map";  // 修改为map坐标系 <<< 关键修改
        
        if(move_client.call(move_srv)) {
            ROS_INFO("Moved backward successfully in map frame");
        } else {
            ROS_WARN("Failed to move backward in map frame");
        }
        
        sleep(1);  // 移动后暂停1秒
    }
}
else if (text.find("我要到") != string::npos) {
                for (int i = 0; i <7; i++) {
                    if (text.find(m_point[i].name.c_str()) != string::npos) {
                        audio.voice_tts("好的，这就带你去");
                        audio.voice_tts(m_goto[i].name1.c_str());
                        audio.goto_nav(&m_goto[i]);
                        audio.voice_tts(m_goto[i].present.c_str());
                        
                        audio.goto_nav(&m_goto[6]); // 导航到充电桩
                        
                        // 充电服务
                        ros::NodeHandle n;
                        ros::ServiceClient relative_move_client = n.serviceClient<relative_move::SetRelativeMove>("relative_move");
                        ros::ServiceClient ar_track_client = n.serviceClient<ar_pose::Track>("track");
                        relative_move::SetRelativeMove RelativeMove_data;
                        ar_pose::Track Track_data;

                        ros::service::waitForService("relative_move");
                        ros::service::waitForService("track");

                        Track_data.request.ar_id = 0;
                        Track_data.request.goal_dist = 0.3;
                        ar_track_client.call(Track_data);
                        
                        RelativeMove_data.request.goal.x = -0.09;
                        RelativeMove_data.request.global_frame = "odom";
                        relative_move_client.call(RelativeMove_data);
                        
                        audio.voice_tts(m_goto[6].present.c_str()); // "充电成功"
                        
                        // 从充电桩脱离
                        RelativeMove_data.request.goal.x = 0.15;
                        RelativeMove_data.request.global_frame = "odom";
                        relative_move_client.call(RelativeMove_data); // 执行脱离移动
                         audio.goto_nav(&m_goto[7]); // 导航到充电桩
                        audio.voice_tts(m_goto[7].name.c_str()); // "返回起点"
                        
                        // 持续尝试导航到起点直到成功
                        int max_retries = 5;
                        int retry_count = 0;
                        bool success = false;
                        while (!success && retry_count < max_retries) {
                            success = audio.goto_nav(&m_goto[8]);
                            if (!success) {
                                ROS_INFO("Navigation to starting point failed, retrying in 5 seconds...");
                                sleep(5);
                                retry_count++;
                            }
                        }
                        if (success) {
                            audio.voice_tts(m_point[8].present.c_str()); // 起点介绍
                        } else {
                            ROS_ERROR("Failed to reach starting point after %d retries.", max_retries);
                            audio.voice_tts("无法返回起点，请检查导航系统。");
                        }
                        break;
                    }
                }
            } else if (text.find("我是谁") != string::npos) {
                ros::NodeHandle n;
                ros::ServiceClient face_rec_client = n.serviceClient<face_rec::recognition_results>("face_recognition_results");
                face_rec::recognition_results face_rec_data;
                ros::service::waitForService("face_recognition_results");
                while (ros::ok()) {
                    face_rec_data.request.mode = 1;
                    face_rec_client.call(face_rec_data);
                    if (face_rec_data.response.success) {
                        int face_num = face_rec_data.response.result.num;
                        cout << "我看到了" << face_num << "个人脸,他们分别是：" << endl;
                        for (int i = 0; i < face_num; i++) {
                            string rec_name = face_rec_data.response.result.face_data[i].name;
                            cout << rec_name << " ";
                            audio.voice_tts(("我看到了" + to_string(face_num) + "个人脸").c_str());
                            audio.voice_tts(("名字是" + rec_name).c_str());
                        }
                        cout << endl;
                    }
                }
            }
        }
    }
    return 0;
}
