#include "Aria.h"
#include <unistd.h>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <pthread.h>
#include <string>
#include <math.h>
using namespace std;

const int MAXSIZE = 1024;
const int THREAD_NUM = 10;
string inst;
string rg_ins;
int socket_fd;
int accept_fd[THREAD_NUM];
sockaddr_in myserver;
sockaddr_in remote_addr;
bool rg[3]; //1-red,0-green
int whichisrgflag = -1;
int flag = 0;
bool rgflag = false; // be true when counter rg traffic
//bool rotate = false;
pthread_t tid[THREAD_NUM];
int thread_count = 0;
int robot_vel = 60;

void tcp_server_init(int listen_port) {

        if(( socket_fd = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP)) < 0 ){
                throw "socket() failed";
        }

        memset(&myserver,0,sizeof(myserver));
        myserver.sin_family = AF_INET;
        myserver.sin_addr.s_addr = htonl(INADDR_ANY);
        myserver.sin_port = htons(listen_port);
        if( bind(socket_fd,(sockaddr*) &myserver,sizeof(myserver)) < 0 ) {
                throw "bind() failed";
        }

        if( listen(socket_fd,THREAD_NUM) < 0 ) {
                throw "listen() failed";
        }
}

void* recv_msg(void* _num) {
        cout << "before" <<endl;
        int num = *((int *)&_num);
        cout << "num=" <<num << endl;
        while(1) {
                int len;
                char buffer[MAXSIZE];
                memset(buffer,0,MAXSIZE);
                if( ( len = recv(accept_fd[num],buffer,MAXSIZE,0)) < 0 ) {
                        throw("Read() error!");
                }
                else if(len>0)
                {
                    flag = 1;
                    if(memcmp(buffer,"00turn",6) == 0)
                    {
                        cout << "into turn" <<endl;
                        whichisrgflag = num;
                        rg_ins = "";
                        for(int i=2;i<len;i++)
                            rg_ins += buffer[i];
                        rgflag = true;
                        for(int i=0;i<3;i++)
                        {
                            if(rg_ins[i+4] == 'r')
                                rg[i] = true;
                            else if(rg_ins[i+4] == 'g')
                                rg[i] = false;
                            else
                            {
                                cout << "shit error,reveive "<<rg_ins <<endl;
                            }
                        }
                        continue;
                    }


                    inst="";
                    for(int i=2;i<len;i++)
                    {
                        inst += buffer[i];
                    }
//                    if(inst.compare(0,3,"turn", 0, 3) == 0)
//                    {
//                        rgflag = true;
//                        for(int i=0;i<3;i++)
//                        {
//                            if(inst[i+4] == 'r')
//                                rg[i] = true;
//                            else if(inst[i+4] == 'g')
//                                rg[i] = false;
//                            else
//                            {
//                                cout << "shit error,reveive "<<inst <<endl;
//                            }
//                        }
//                    }

                }
}
}

void* accept_link(void*) {
    socklen_t sin_size = sizeof(struct sockaddr_in);
    while(1){
        if(( accept_fd[thread_count] = accept(socket_fd,(struct sockaddr*) &remote_addr,&sin_size)) == -1 )
        {
                throw "Accept error!";
        }
        printf("Received a connection from %s\n",(char*) inet_ntoa(remote_addr.sin_addr));
        pthread_create(&tid[thread_count], NULL, &recv_msg, (void*)thread_count);
        pthread_detach(tid[thread_count]);
        ++thread_count;
    }

}




void str2int(int &int_temp,const string &string_temp)
{
    stringstream stream(string_temp);
    stream>>int_temp;
}

void RobotRotate(int angle, ArRobot &r)
{
    r.setVel(0);
    r.setDeltaHeading(angle);
    int tmpcount = 0;
    do
    {
        sleep(1);
        tmpcount++;
         cout << "in rotate" << endl;

    }while(!r.isHeadingDone() && tmpcount <10);
}


int main(int argc,char **argv)
{
    memset(rg,0,sizeof(rg));
    tcp_server_init(8888);
    pthread_t thr;
    pthread_create(&thr,NULL,&accept_link,NULL);
    pthread_detach(thr);
    Aria::init(); //初始化
    argc = 3;
    argv[0] = "";
    argv[1] = "-rp";
    argv[2] = "/dev/ttyUSB0";
    ArArgumentParser parser(&argc,argv);
    parser.loadDefaultArguments();
    ArRobot robot;
    ArRobotConnector connector(&parser,&robot);
    if (!connector.connectRobot())
    {
        if (!parser.checkHelpAndWarnUnparsed())
        {
            ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
        }
        else
        {
            ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    if(!robot.isConnected())
    {
        ArLog::log(ArLog::Terse, "Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
    }

    if(!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
        return 1;
    }

    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);
    printf("You may press escape to exit\n");

    robot.runAsync(true); //启动机器人线程
    ArUtil::sleep(1000);
    robot.lock();
    robot.comInt(ArCommands::ENABLE,1);
    robot.unlock();

    //robot.setAbsoluteMaxRotVel(15);
    robot.setRotVelMax(15);
    while(1)
    {
        if(flag == 1){
            double rotate_temp = 89 + (rand()%20)*1.0/10;
            flag = 0;
            //cout <<"rotate_tmp = "<<rotate_temp << endl;
            cout << rg[0] << rg[1] << rg[2] <<endl;
            cout << "sub="<<inst.substr(0,3)<<endl;
            if(inst == "forward"){
                if(rg[1])
                    continue;
                if(rgflag)
                {
                    rgflag = false;
                    memset(rg,0,sizeof(rg));
                }
                cout << "receive forward" << endl;
                robot.setVel(robot_vel);
            }
            else if(inst == "back")
            {
                cout << "receive back" << endl;
                robot.setVel(-200);
            }
            else if(inst == "stop" || rg_ins == "turnrrr"){
                cout << "receive stop" << endl;
                robot.stop();
                rg_ins = "";
                continue;
            }
            else if(inst == "left"){
                if(rg[0])
                    continue;
                if(rgflag)
                {
                    rgflag = false;
                    memset(rg,0,sizeof(rg));
                }
                cout << "receive left" << endl;
                RobotRotate(rotate_temp, robot);
//                robot.setVel(0);

//                robot.setDeltaHeading(rotate_temp);
//                int tmpcount = 0;
//                do
//                {
//                    sleep(1);
//                    tmpcount++;
//                     cout << "in left" << endl;

//                }while(!robot.isHeadingDone() && tmpcount <10);
                sendMessage("haveturn");
            }
            else if(inst == "right"){
                if(rg[2])
                    continue;
                if(rgflag)
                {
                    rgflag = false;
                    memset(rg,0,sizeof(rg));
                }
                cout << "receive right" << endl;
//                robot.setVel(0);
//                robot.setDeltaHeading(-rotate_temp);
//                int tmpcount = 0;
//                do
//                {
//                    sleep(1);
//                    tmpcount++;
//                     cout << "in right" << endl;
//                }while(!robot.isHeadingDone() && tmpcount < 10);
                RobotRotate(-rotate_temp, robot);
                sendMessage("haveturn");
            }
            else if(inst == "exit"){
                cout << "receive exit" << endl;
                break;
            }
            else if(inst == "reset"){
                memset(rg,0,sizeof(rg));
                rgflag = false;
                rg_ins = "";
                cout << "receive reset" << endl;
            }

            else if(inst.substr(0,8) == "obstacle")
            {
                int angle;
                str2int(angle,inst.substr(8));
                cout << "encounter obstacle" << endl;
                cout << "angle=" << angle << endl;
                RobotRotate(angle,robot);
                int distance = 1000 / cos(angle);
                robot.move(distance);
                while(!robot.isMoveDone())
                {
                    sleep(1);
                    cout << "is avoid obstacle" <<endl;
                }
                RobotRotate(-angle,robot);
                robot.move(1500);
                RobotRotate(-angle,robot);
                robot.move(distance);
                while(!robot.isMoveDone())
                {
                    sleep(1);
                    cout << "is avoid obstacle" <<endl;
                }
                RobotRotate(angle,robot);
                robot.setVel(robot_vel);
            }

            else if(inst.substr(0,3) == "set")
            {
                if(inst.substr(3,3) == "vel")
                {
                    int vel;
                    str2int(vel, inst.substr(6, inst.length()-6));
                    cout << "set vel = "<<vel <<endl;
                    robot_vel = vel;
                }
                else if(inst.substr(3,5) == "angle")
                {
                    int angle;
                    str2int(angle, inst.substr(8, inst.length()-8));
                    cout << "set angle = "<< angle <<endl;
                    RobotRotate(angle,robot);

                }
                else if(inst.substr(3,6) == "rotvel")
                {
                    int rotvel;
                    str2int(rotvel, inst.substr(9, inst.length()-9));
                    //robot.setAbsoluteMaxRotVel(rotvel);
                    robot.setRotVelMax(15);
                    cout << "set rotvel = "<<rotvel <<endl;
                }
            }
            else{
                cout << "error, inst= " << inst <<" rg_ins="<<rg_ins<< endl;
            }
            inst = "";
        }
        ArUtil::sleep(200);
    }

    robot.lock();
    robot.disconnect(); //断开连接
    robot.unlock();
    Aria::shutdown(); //退出

    close(socket_fd);
    //close(accept_fd);
    return 0;
}
