
#include <ros/ros.h>
#include <ros/network.h>
#include "robot.h"
#include "ros_launch_manager.hpp"
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <QThread>
#include <QProcess>
#include <QVector>
#include <QString>

//int slam_pid = -1;
//int nav_pid = -1;
//int map_pid = -1;
//ROSLaunchManager *roslaunch_slam=NULL;
//ROSLaunchManager *roslaunch_nav=NULL;
//ROSLaunchManager *roslaunch_map=NULL;

//void launchBack(const robot_msg::robot &msg){
////    ROS_INFO("%s",msg.comparams.size());
//    std::string args_string = std::accumulate(std::next(std::begin(msg.comparams)), std::end(msg.comparams), msg.comparams[0], [](std::string lhs, std::string rhs) -> std::string { return lhs + " " + rhs; });
//    ROS_INFO("receive %s",args_string.c_str());
//    std::vector<std::string> v(msg.comparams.begin(),msg.comparams.end());
//    std::vector<char*> vec(msg.comparams.size());
//    for(int i=0; i<v.size();i++){
//        vec[i] = v[i].data();
//    }
////    vec.insert(vec.begin(),"roslaunch");
//    if(msg.type == "slam"){
//        if(nav_pid!=-1){
//            try {
//                roslaunch_nav->stop(nav_pid, SIGINT);
//                nav_pid=-1;
//            }
//            catch (std::exception const &exception) {
//                ROS_WARN("%s", exception.what());
//            }
//        }
//        if(roslaunch_slam==NULL) roslaunch_slam = new ROSLaunchManager;
//        try {
//            if(slam_pid==-1 )
//                slam_pid = roslaunch_slam->start(
//                        vec);
//        }
//        catch (std::exception const &exception) {
//            ROS_WARN("%s", exception.what());
//        }
//    }
//    else if(msg.type == "nav"){
//        if(slam_pid!=-1){
//            try {
//                roslaunch_slam->stop(slam_pid, SIGINT);
//                slam_pid = -1;
//            }
//            catch (std::exception const &exception) {
//                ROS_WARN("%s", exception.what());
//            }
//        }
//        if(roslaunch_nav==NULL) roslaunch_nav = new ROSLaunchManager;
//        try {
//            nav_pid = roslaunch_nav->start(
//                    vec);
//        }
//        catch (std::exception const &exception) {
//            ROS_WARN("%s", exception.what());
//        }
//    }
//    else if(msg.type == "savemap"){
//
//        if (roslaunch_map == NULL) roslaunch_map = new ROSLaunchManager;
//        try {
//            map_pid = roslaunch_nav->start(
//                    vec);
//        }
//        catch (std::exception const &exception) {
//            ROS_WARN("%s", exception.what());
//        }
//    }
//
//}

class ROSCMD:QThread{
public:
    ROSCMD(int argc, char** argv):init_argc(argc), init_argv(argv){
        test = std::shared_ptr<QProcess>(new QProcess);
        slam = std::shared_ptr<QProcess>(new QProcess);
        nav = std::shared_ptr<QProcess>(new QProcess);
    }

    ~ROSCMD(){
        std::cout << "kill node\n";
        if(ros::isStarted()){
            if(slam->state()==QProcess::Running){
                ::kill(slam->processId(), SIGINT);
               test->terminate();
               test->waitForFinished();
            }
            if (nav->state()==QProcess::Running){
                ::kill(nav->processId(),SIGINT);
               nav->terminate();
               nav->waitForFinished();
            }
            if(test->state()==QProcess::Running){
                ::kill(test->processId(),SIGINT);
               test->terminate();
               test->waitForFinished();
            }

            ros::shutdown();    // 关闭节点
            ros::waitForShutdown();
        }
//        wait(); //等待结束
    }



public:

//    bool init(){
//        std::cout << "init node\n";
//        std::cout <<init_argc << " " <<init_argv << std::endl;
//        ros::init(init_argc, init_argv, "launch_program");
//        if(!ros::master::check()){
//            ROS_WARN("ros master has not started yet!!!");
//            return false;
//        }
//        std::cout << "connect master successful\n";
//        ros::start();
//        ros::NodeHandle n;
//        sub = n.subscribe("slam_nav", 100, &ROSCMD::launch, this);
////        start();    // 启动 run函数，多线程
//        return true;
//    }

    void launch(const robot_msg::robotConstPtr &msg);

    void run(){
//        std::cout << "run node\n";
//        ros::Rate r(10);
//        while(ros::ok()){
//            ros::spinOnce();
//            r.sleep();
//        }
    }

private:
    std::shared_ptr<QProcess> test;
    std::shared_ptr<QProcess> slam;
    std::shared_ptr<QProcess> nav;
    ros::Subscriber sub;

    int init_argc;
    char** init_argv;
};



void ROSCMD::launch(const robot_msg::robotConstPtr &msg){
    // 拼凑为一个字符串，用空格间隔
    std::string args_string = std::accumulate(std::next(std::begin(msg->comparams)), std::end(msg->comparams), msg->comparams[0], [](std::string lhs, std::string rhs) -> std::string { return lhs + " " + rhs; });
    ROS_INFO("receive type:%s, execute %s", msg->type.c_str(), args_string.c_str());

    QVector<QString> qv;
    std::transform(msg->comparams.begin()+1, msg->comparams.end(), std::back_inserter(qv),
                   [](const std::string &v){ return QString::fromStdString(v); });
    QStringList cmdList=qv.toList();

    if(msg->type=="close" or msg->type=="kill"){
        if(test->state() == QProcess::Running){
            ::kill(test->processId(),SIGINT);
            test->terminate();
            test->waitForFinished();
        }
        return ;
    }
    if(msg->type=="killall" || msg->type=="closeall"){
        std::cout << "kill all process node\n";
        if(test->state() == QProcess::Running){
            ::kill(test->processId(),SIGINT);
            test->terminate();
            test->waitForFinished();
        }
        if(slam->state()==QProcess::Running){
            ::kill(slam->processId(),SIGINT);
            slam->terminate();
            slam->waitForFinished();
        }
        if(nav->state()==QProcess::Running){
            ::kill(nav->processId(),SIGINT);    // qprocess 无法关闭子进程
            nav->terminate();
            nav->waitForFinished();
        }
        return ;
    }

    if(msg->comparams.empty() || msg->comparams[0]=="killall" || msg->comparams[0]=="closeall"){
        std::cout << "cmd list is null\n";
        return ;
    }
        
    if(msg->type=="slam"){   // slam 与 navigation 不能同时启动
        if(nav->state()==QProcess::Running){
            ROS_INFO("kill navigation through pid %lld", nav->processId());
            ::kill(nav->processId(),SIGINT);
            nav->terminate();
            nav->waitForFinished();
        }
        if(slam->state()==QProcess::Running) slam->close() ;
        slam->start(QString::fromStdString(msg->comparams[0]),cmdList);//qvec);
        slam->waitForStarted();
        ROS_INFO("start slam through pid %lld", slam->processId());
    }
    else if(msg->type=="nav"){
        if(slam->state()==QProcess::Running){
            ROS_INFO("kill slam through pid %lld", slam->processId());
            ::kill(slam->processId(),SIGINT);
            slam->terminate();
            slam->waitForFinished();
        }
        if (nav->state()==QProcess::Running) {
            nav->close();
        }
        nav->start(QString::fromStdString(msg->comparams[0]),cmdList);//qvec);
        nav->waitForStarted();
        ROS_INFO("start navigation through pid %lld", nav->processId());
    }
    else{
        if(test->state()==QProcess::Running){
            ROS_INFO("kill test through pid %lld", test->processId());
            ::kill(test->processId(),SIGINT);
            test->terminate();
            test->waitForFinished();
        }
//        if (nav->state()==QProcess::Running) return ;
        test->start(QString::fromStdString(msg->comparams[0]),cmdList);//qvec);
        test->waitForStarted();
        ROS_INFO("start %s through pid %lld",msg->comparams[0].c_str(), test->processId());
    }
}




int main(int argc,char **argv){
    ros::init(argc,argv,"launch_program");
    ROSCMD roscmd(argc,argv);
//    roscmd.init();
//    roscmd.run();


//    ros::start();
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("slam_nav", 100, &ROSCMD::launch, &roscmd);
    ros::Rate r(10);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
    }
}
