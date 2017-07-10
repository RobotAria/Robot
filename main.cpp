#include "Aria.h"
#include <unistd.h>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <string>

using namespace std;

const int MAXSIZE = 1024;
const int THREAD_NUM = 10;
string inst;

int socket_fd;
int accept_fd[THREAD_NUM];
sockaddr_in myserver;
sockaddr_in remote_addr;
bool rg[3]; //1-red,0-green

int flag = 0;
bool rgflag = false; // be true when counter rg traffic
bool rotate = false;
pthread_t tid[THREAD_NUM];
int thread_count = 0;

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
                } else if(len>0){
                        flag = 1;

                        inst="";
                        for(int i=2;i<len;i++)
                        {
                            inst += buffer[i];
                        }
                        if(inst.compare(0,3,"turn", 0, 3))
                        {
                            rgflag = true;
                            for(int i=0;i<3;i++)
                            {
                                if(inst[i+4] == 'r')
                                    rg[i] = true;
                                else if(inst[i+4] == 'g')
                                    rg[i] = false;
                                else
                                {
                                    cout << "shit erroe,reveive "<<inst <<endl;
                                }
                            }
                        }

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
        cout << "1" <<endl;
        pthread_detach(tid[thread_count]);
        cout << "2" <<endl;
        ++thread_count;
    }

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

    while(1){
        double rotate_temp = 89 + (rand()%20)*1.0/10;
        if(flag == 1){
            if(rgflag)
                continue;
            flag = 0;
            cout << flush;
            if(inst == "forward"){
                if(rg[1])
                    continue;
                cout << "receive forward" << endl;
                robot.setVel(1);
            }
            else if(inst == "stop"){
                cout << "receive stop" << endl;
                robot.stop();
            }
            else if(inst == "left"){
                if(rg[0])
                    continue;
                cout << "receive left" << endl;
                robot.setDeltaHeading(rotate_temp);
                int tmpcount = 0;
                do
                {
                    sleep(1);
                    tmpcount++;
                     cout << "in left" << endl;

                }while(!robot.isHeadingDone() && tmpcount <5);
            }
            else if(inst == "right"){
                if(rg[2])
                    continue;
                cout << "receive right" << endl;
                robot.setDeltaHeading(-rotate_temp);
                int tmpcount = 0;
                do
                {
                    sleep(1);
                    tmpcount++;
                     cout << "in right" << endl;
                }while(!robot.isHeadingDone() && tmpcount <5);
            }
            else if(inst == "exit"){
                cout << "receive exit" << endl;
                break;
            }
            else{
                cout << "error, receive " << inst << endl;
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
