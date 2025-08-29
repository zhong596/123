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

// 扩展 m_point 数组以匹配 m_goto，确保索引安全
struct Point m_point[10] = {
    {1.040, 2.132, -0.707, 0.707, "上海", "上上海馆", " 上上海是中国的经济中心"},
    {1.050, 1.171, 1.000, 1.000, "深圳", "深深圳馆", " 深深圳是中国的科创中心"},
    {2.523, 2.132, -0.707, 0.707, "吉林", "吉吉林馆", " 吉吉林位于我国东北是人参之都"},
    {2.521, 1.126, -0.707, 0.707, "广州", "广广州馆", " 广广州自古都是我国的商业之都"},
    {2.558, 0.079, -0.707, 0.707, "北京", "北北京馆", " 北北京是中国的首都，政治中心"},
    {2.000, 0.079, 1.000, 0.000, "起点", "其", "点"},
    {0.000, -0.089, -1.000, 1.000, "起点", "起点", " 起始点"},
    {0.000, -0.089, 0.000, 1.000, "起点", "起点", " 充电完成，返回起点"},
    {-0.060, -0.059, 0.000, 1.000, "起点", "起点", " 欢迎回到起点"}
};

// m_goto 数组保持不变
struct Point m_goto[10] = {
    {1.020, 2.132, 1.000, 1.000, "上海", "上上海馆", " 上上海是中国的经济中心"},
    {1.050, 1.171, 1.000, 1.000, "深圳", "深深圳馆", " 深深圳是中国的科创中心"},
    {2.523, 2.132, 1.000, 1.000, "吉林", "吉吉林馆", " 吉吉林位于我国东北是人参之都"},
    {2.521, 1.126, 1.000, 1.000, "广州", "广广州馆", " 广广州自古都是我国的商业之都"},
    {2.558, 0.079, 1.000, 1.000, "北京", "北北京馆", " 北北京是中国的首都，政治中心"},
    {0.000, -0.059, 0.000, 1.000, "起点", "起点", " 起始点"},
    {0.430, 2.000, -0.707, 0.707, "充电桩", "充电桩", " 充电成功"},
    {0.020, -0.049, -1.000, 1.000, "起点", "起点", " 欢迎回到起点"},
     {0.020, -0.049, 0.000, 1.000, "起点", "起点", " 欢迎回到起点"}
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> AC;

class Interaction {
public:
    Interaction();
    void reinitialize_services(); // 添加服务重新初始化方法
    string voice_collect();
    string voice_dictation(const char* filename);
    string voice_tts(const char* text);
    bool m_goto_nav(struct Point* point);
private:
    ros::NodeHandle n;
    ros::ServiceClient collect_client, dictation_client, tts_client;
};

Interaction::Interaction() {
    reinitialize_services();
}

void Interaction::reinitialize_services() {
    collect_client = n.serviceClient<robot_audio::Collect>("voice_collect");
    dictation_client = n.serviceClient<robot_audio::robot_iat>("voice_iat");
    tts_client = n.serviceClient<robot_audio::robot_tts>("voice_tts");
}

string Interaction::voice_collect() {
    if (!ros::service::waitForService("voice_collect", ros::Duration(5.0))) {
        ROS_ERROR("voice_collect service unavailable");
        return "";
    }
    robot_audio::Collect srv;
    srv.request.collect_flag = 1;
    if (collect_client.call(srv)) {
        return srv.response.voice_filename;
    } else {
        ROS_ERROR("Failed to call voice_collect service");
        return "";
    }
}

string Interaction::voice_dictation(const char* filename) {
    if (!ros::service::waitForService("voice_iat", ros::Duration(5.0))) {
        ROS_ERROR("voice_iat service unavailable");
        return "";
    }
    robot_audio::robot_iat srv;
    srv.request.audiopath = filename;
    if (dictation_client.call(srv)) {
        return srv.response.text;
    } else {
        ROS_ERROR("Failed to call voice_iat service");
        return "";
    }
}

string Interaction::voice_tts(const char* text) {
    if (!ros::service::waitForService("voice_tts", ros::Duration(5.0))) {
        ROS_ERROR("voice_tts service unavailable");
        return "";
    }
    robot_audio::robot_tts srv;
    srv.request.text = text;
    if (tts_client.call(srv)) {
        string cmd = "play " + srv.response.audiopath;
        system(cmd.c_str());
        sleep(1);
        return srv.response.audiopath;
    } else {
        ROS_ERROR("Failed to call voice_tts service");
        return "";
    }
}

bool Interaction::m_goto_nav(struct Point* point) {
    AC* ac = new AC("move_base", true);
    ROS_INFO("Waiting for action server to start.");
    if (!ac->waitForServer(ros::Duration(10.0))) {
        ROS_ERROR("Navigation server not available");
        delete ac;
        return false;
    }
    ROS_INFO("Action server started, sending goal.");
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = point->x;
    goal.target_pose.pose.position.y = point->y;
    goal.target_pose.pose.orientation.z = point->z;
    goal.target_pose.pose.orientation.w = point->w;
    ac->sendGoal(goal);
    ac->waitForResult(ros::Duration(60.0));
    bool success = (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (success)
        ROS_INFO("Goal succeeded!");
    else
        ROS_INFO("Goal failed!");
    delete ac;
    return success;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "interaction");
    Interaction audio;
    string dir, text;
    
    while (ros::ok()) {
        dir = audio.voice_collect();
        if (dir.empty()) {
            ROS_WARN("Voice collection failed, retrying in 2 seconds");
            sleep(2);
            audio.reinitialize_services();
            continue;
        }
        
        text = audio.voice_dictation(dir.c_str());
        if (text.empty()) {
            ROS_WARN("Voice dictation failed, retrying in 2 seconds");
            sleep(2);
            audio.reinitialize_services();
            continue;
        }
        
        if (text.find("元宝元宝") != string::npos) {
            audio.voice_tts("  哎，什么事呀");
            dir = audio.voice_collect();
            text = audio.voice_dictation(dir.c_str());
            
            if (text.find("带我参观一圈") != string::npos) {
                audio.voice_tts("  好的，这就带您参观");
                for (int i = 0; i <= 7; i++) { // 修正语法错误
                    audio.m_goto_nav(&m_point[i]);
                    audio.voice_tts(m_point[i].present.c_str());
                }
            } 
            else if (text.find("我要到") != string::npos) {
                bool found_destination = false;
                for (int i = 0; i <= 4; i++) { // 只循环5个目的地
                    if (text.find(m_point[i].name.c_str()) != string::npos) {
                        found_destination = true;
                        audio.voice_tts("好的，这就带你去");
                        audio.voice_tts(m_goto[i].name1.c_str());
                        audio.m_goto_nav(&m_goto[i]);
                        audio.voice_tts(m_goto[i].present.c_str());
                        
                        // 导航到充电桩
                        
                        audio.m_goto_nav(&m_goto[6]);
                        
                        // 充电服务
                        ros::NodeHandle n;
                        ros::ServiceClient relative_move_client = n.serviceClient<relative_move::SetRelativeMove>("relative_move");
                        ros::ServiceClient ar_track_client = n.serviceClient<ar_pose::Track>("track");
                        
                        bool charging_success = false;
                        if (ros::service::waitForService("relative_move", ros::Duration(5.0)) && 
                            ros::service::waitForService("track", ros::Duration(5.0))) {
                            ar_pose::Track Track_data;
                            Track_data.request.ar_id = 0;
                            Track_data.request.goal_dist = 0.3;
                            
                            if (ar_track_client.call(Track_data)) {
                                ROS_INFO("AR track successful");
                                relative_move::SetRelativeMove RelativeMove_data;
                                RelativeMove_data.request.goal.x = -0.09;
                                RelativeMove_data.request.global_frame = "odom";
                                
                                if (relative_move_client.call(RelativeMove_data)) {
                                    audio.voice_tts(m_goto[6].present.c_str()); // "充电成功"
                                    sleep(5); // 模拟充电时间
                                    RelativeMove_data.request.goal.x = 0.15;
                                    RelativeMove_data.request.global_frame = "odom";
                                    relative_move_client.call(RelativeMove_data);
                                    charging_success = true;
                                }
                            }
                        }
                        
                        if (!charging_success) {
                            audio.voice_tts("充电失败");
                        }
                        
                        // 导航回起点
                       
                        int max_retries = 5;
                        bool success = false;
                        for (int retry = 0; retry < max_retries && !success; retry++) {
                            success = audio.m_goto_nav(&m_goto[7]);
                            if (!success) {
                                ROS_INFO("Navigation to starting point failed, retrying in 5 seconds...");
                                sleep(5);
                            }
                        }
                        if (success) {
                            audio.voice_tts(m_point[7].present.c_str());
                        } else {
                            ROS_ERROR("Failed to reach starting point after %d retries.", max_retries);
                            audio.voice_tts("无法返回起点，请检查导航系统。");
                        }
                        audio.m_goto_nav(&m_goto[8]);
                        // 重新初始化服务，确保语音功能恢复
                        audio.reinitialize_services();
                        break;
                    }
                }
                if (!found_destination) {
                    audio.voice_tts("抱歉，我没有找到您要去的地方");
                }
            } 
            else if (text.find("我是谁") != string::npos) {
                ros::NodeHandle n;
                ros::ServiceClient face_rec_client = n.serviceClient<face_rec::recognition_results>("face_recognition_results");
                if (!ros::service::waitForService("face_recognition_results", ros::Duration(5.0))) {
                    audio.voice_tts("人脸识别服务不可用");
                    continue;
                }
                
                face_rec::recognition_results face_rec_data;
                int max_attempts = 3;
                int attempts = 0;
                bool recognized = false;
                while (ros::ok() && attempts < max_attempts && !recognized) {
                    face_rec_data.request.mode = 1;
                    if (face_rec_client.call(face_rec_data) && face_rec_data.response.success) {
                        int face_num = face_rec_data.response.result.num;
                        if (face_num > 0) {
                            string response = "我看到了" + to_string(face_num) + "个人脸";
                            audio.voice_tts(response.c_str());
                            for (int i = 0; i < face_num; i++) {
                                string rec_name = face_rec_data.response.result.face_data[i].name;
                                string name_response = "名字是" + rec_name;
                                audio.voice_tts(name_response.c_str());
                            }
                            recognized = true;
                        } else {
                            audio.voice_tts("我没有识别到人脸");
                        }
                    } else {
                        audio.voice_tts("人脸识别失败");
                    }
                    attempts++;
                    sleep(2);
                }
                if (!recognized) {
                    audio.voice_tts("无法识别，请稍后再试");
                }
            }
        }
        ros::spinOnce();
    }
    return 0;
}
