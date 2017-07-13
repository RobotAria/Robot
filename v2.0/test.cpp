#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

using namespace std;
using namespace cv;

enum
{
    Processor_cl,
    Processor_gl,
    Processor_cpu
};

Mat rgbmat, depthmat, irmat, graymat, hsv;

vector<Vec3f> circles;
int flag[256];
bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
    protonect_shutdown = true;
}

double dist(double x1, double y1, double x2, double y2)		//计算圆心间距离判断是否重叠
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void ChangeColor(cv::Mat *p,int x,int y,int r,int g,int b)		//改变图中的颜色信息
{
    p->data[(x*p->cols+y)*3]=r;
    p->data[(x*p->cols+y)*3+1]=g;
    p->data[(x*p->cols+y)*3+2]=b;
}

CvScalar Get(cv::Mat *p, int x, int y)		//获得某个点的颜色信息
{
    CvScalar t;
    t.val[0] = p->data[(x*p->cols+y)*3];
    t.val[1] = p->data[(x*p->cols+y)*3+1];
    t.val[2] = p->data[(x*p->cols+y)*3+2];
    return t;
}

size_t Floodfill(size_t x)		//使用floodfill算法对圆重叠进行判断
{
    size_t now = x;
    flag[(int)x] = 2;
    for (size_t i = 0; i < circles.size(); i++)
        if (flag[(int)i] == 0)
        {
            double Dis=dist(circles[i][0],circles[i][1],circles[x][0],circles[x][1]);
            if (Dis < circles[x][2] || Dis < circles[i][2])
            {
                size_t y = Floodfill(i);
                if (circles[y][2] > circles[now][2])
                    now = y;
            }
        }
    return now;
}

CvScalar Color(double xx, double yy, double rr)   //计算圆内有效颜色的中位数得到颜色
{
    int r=cvRound(rr);
    CvScalar now;
    int sumH[256], sumS[256], sumV[256];
    int sum;
    for (int i = 0; i < 255; i ++)
        sumH[i]=sumS[i]=sumV[i]=0;
    sum=0;

    for (int i = -r+1; i < r; i++)
    {
        int p = (int)sqrt(r*r-i*i);
        for (int j = -p+1; j < p; j++)
        {
            if (cvRound(xx)+j > hsv.cols || cvRound(xx)+j < 0) continue;
            if (cvRound(yy)+i > hsv.rows || cvRound(yy)+i < 0) continue;
            now = Get(&hsv, cvRound(yy)+i, cvRound(xx)+j);
            if (now.val[0] == 0 && now.val[1] == 0 && now.val[2] == 0) continue;
            sum ++;
            sumH[(int)now.val[0]] ++;
            sumS[(int)now.val[1]] ++;
            sumV[(int)now.val[2]] ++;
        }
    }

    int p;
    CvScalar mid;
    for (mid.val[0]=p=0;mid.val[0] < 255 && p<sum/2;mid.val[0]++,p+=sumH[(int)mid.val[0]]);
    for (mid.val[1]=p=0;mid.val[1] < 255 && p<sum/2;mid.val[1]++,p+=sumS[(int)mid.val[1]]);
    for (mid.val[2]=p=0;mid.val[2] < 255 && p<sum/2;mid.val[2]++,p+=sumV[(int)mid.val[2]]);

    return mid;
}


double isCircle(double xx, double yy, double rr)   //计算圆边界有效颜色占比，用来判断是否是圆
{
    int r=cvRound(rr);
    CvScalar now;
    int sum=0, sumAll=0;
    for (int i = -r+1; i < r; i++)
    {
        int p = (int)sqrt(r*r-i*i);
        for (int j = -p+1; j < p; j++)
        {
            if (cvRound(xx)+j > hsv.cols || cvRound(xx)+j < 0) continue;
            if (cvRound(yy)+i > hsv.rows || cvRound(yy)+i < 0) continue;
            sumAll += 1;
            now = Get(&hsv, cvRound(yy)+i, cvRound(xx)+j);
            if (now.val[0] == 0 && now.val[1] == 0 && now.val[2] == 0) continue;
            sum += 1;
            if (j == -p+3 && j < p-4) j = p-4;
        }
    }
    return (double(sum)/double(sumAll));
}

double Per(double xx, double yy, double rr)   //计算圆中有效颜色点占比
{
    int r=cvRound(rr);
    CvScalar now;
    int sum=0, sumAll=0;
    for (int i = -r+1; i < r; i++)
    {
        int p = (int)sqrt(r*r-i*i);
        for (int j = -p+1; j < p; j++)
        {
            if (cvRound(xx)+j > hsv.cols || cvRound(xx)+j < 0) continue;
            if (cvRound(yy)+i > hsv.rows || cvRound(yy)+i < 0) continue;
            sumAll += 1;
            now = Get(&hsv, cvRound(yy)+i, cvRound(xx)+j);
            if (now.val[0] == 0 && now.val[1] == 0 && now.val[2] == 0) continue;
            sum += 1;
        }
    }
    return double(sum)/double(sumAll);
}


double Dist(float *dep, double xx, double yy, double rr)   //计算圆与kinect距离的中位数得到距离信息
{
    int r=cvRound(rr);
    double sumDep = 0, sumD = 0;
    for (int i = -r+1; i < r; i++)
    {
        int p = (int)sqrt(r*r-i*i);
        for (int j = -p+1; j < p; j++)
        {
            int x, y;
            x = cvRound(yy)+i;
            y = cvRound(xx)+j;
            if (x > depthmat.rows || x < 0) continue;
            if (y > depthmat.cols || y < 0) continue;
            if (!isinf(dep[x*depthmat.cols+y]) && !isnan(dep[x*depthmat.cols+y])
                    && dep[x*depthmat.cols+y] > 0 && dep[x*depthmat.cols+y] < 5000.0)
            {
                sumDep += dep[x*depthmat.cols+y];
                sumD+=1;
            }
        }
    }
    return sumDep/(sumD+1e-8);
}

struct type
{
    int RorG;
    double Dis;
    double x, y, r;
};

int cmp(type x, type y)
{
    return x.x < y.x;
}

double sqr(double x)
{
    return x*x;
}

int main()
{
    std::cout << "Hello World!" << std::endl;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = NULL;
    libfreenect2::PacketPipeline  *pipeline = NULL;

    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;

#if 1 // sean
    int depthProcessor = Processor_cl;

    if(depthProcessor == Processor_cpu)
    {
        if(!pipeline)
            //! [pipeline]
            pipeline = new libfreenect2::CpuPacketPipeline();
        //! [pipeline]
    }
    else if (depthProcessor == Processor_gl) // if support gl
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if(!pipeline)
        {
            pipeline = new libfreenect2::OpenGLPacketPipeline();
        }
#else
        std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if (depthProcessor == Processor_cl) // if support cl
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!pipeline)
            pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
        std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }

    if(pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    signal(SIGINT, sigint_handler);
    protonect_shutdown = false;

    libfreenect2::SyncMultiFrameListener listener(
            libfreenect2::Frame::Color |
            libfreenect2::Frame::Depth |
            libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);
    // sean: that is a driver bug
    // check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger

    unsigned short port = 8888;             // 服务器的端口号
    char *server_ip = "10.129.244.49";       // 服务器ip地址

    int sockfd;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);// 创建通信端点：套接字
    if(sockfd < 0)
    {
        perror("socket");
        exit(-1);
    }
    // 设置服务器地址结构体
    struct sockaddr_in server_addr;
    bzero(&server_addr,sizeof(server_addr)); // 初始化服务器地址
    server_addr.sin_family = AF_INET;   // IPv4
    server_addr.sin_port = htons(port); // 端口
    inet_pton(AF_INET, server_ip, &server_addr.sin_addr.s_addr);    // ip

    int err_log = connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if(err_log != 0)
    {
        perror("connect");
        close(sockfd);
        exit(-1);
    }

    char send_buf[512] = {0};
    char recv_buf[512] = {0};
    strcpy(send_buf, "00turnggg");
    send(sockfd, send_buf, strlen(send_buf), 0);   // 向服务器发送信息

    cv::namedWindow("rgb", WND_PROP_ASPECT_RATIO);
    cv::namedWindow("gray", WND_PROP_ASPECT_RATIO);
    cv::namedWindow("hsv", WND_PROP_ASPECT_RATIO);

    int k = 25;	//调整参数以得到圆
    int minR = 20;
    int minDis = 2;

    while(!protonect_shutdown)
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        //cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(depthmat);

        float *dep = (float*)depthmat.data;		//将深度图信息放到dep数组中
        cv::cvtColor(rgbmat, hsv, CV_BGR2HSV);	//bgr图转hsv图

        cv::imshow("gray", hsv);
        CvScalar now;
	
        //通过颜色对hsv图进行过滤
        for (int i=0;i<hsv.rows;i++)
            for (int j=0;j<hsv.cols;j++)
            {
                now = Get(&hsv, i, j);
                /*
                if (!((now.val[0]<180 && now.val[0]>165
                    //&& now.val[1]<240 && now.val[1]>10
                    //&& now.val[2]<240 && now.val[2]>10
                       )
                    || (now.val[0]<70 && now.val[0]>60
                    && now.val[1]<200 && now.val[1]>20
                    && now.val[2]<200 && now.val[2]>20
                    )))
                */
                if (now.val[0]<185 && now.val[0]>160)
                    ChangeColor(&hsv,i,j,245,255,255);
                else if (now.val[0]<73 && now.val[0]>57
                    //&& now.val[1]<240 && now.val[1]>15
                    //&& now.val[2]<240 && now.val[2]>15
                    )
                    ChangeColor(&hsv,i,j,250,255,255);
                else
                    ChangeColor(&hsv,i,j,0,0,0);
            }

        // 腐蚀操作
        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                             Size( 7*MORPH_ELLIPSE + 1, 7*MORPH_ELLIPSE+1 ),
                                             Point( MORPH_ELLIPSE, MORPH_ELLIPSE ) );
        erode(hsv , hsv, element );
        cv::cvtColor(hsv, graymat, CV_BGR2GRAY);
        GaussianBlur(graymat, graymat, Size(7, 7), 2, 2);
        //霍夫圆
        int minDis_, k_, minR_;
        minDis_ = 10;
        minR_ = 100;
        k_ = 25;
        while (minDis_ > 0)
        {
            circles.clear();
            HoughCircles(graymat, circles, CV_HOUGH_GRADIENT, 1, minDis_, 240, k_, minR_, 1000);
            if ((int)circles.size() < 10)
            {
                minDis_ --;
                minR_ -= 10;
                if (minDis_ > 0)
                {
                    minDis_ --;
                    minR_ -= 10;
                }
                if (minDis_ >7)
                {
                    minDis_ --;
                    minR_ -= 10;
                }
            }
            else break;
        }
        if ((int)circles.size() > 150) circles.clear();
        /*
        while (1)
        {
            HoughCircles(graymat, circles, CV_HOUGH_GRADIENT, 1, 5, 240, k, 25, 500);
            if (T> 100) break;
            if ((int)circles.size() < 20){T+=2;circles.clear();k--;}
            else if ((int)circles.size() > 50){T+=10;circles.clear();k+=5;}
            else break;
        }
        */
        printf("%d\n" ,(int)circles.size());
        for (int i=0; i<256; i++)
            flag[i]=0;

        //通过有效颜色在圆中/圆边界占比，判断是否应该舍弃该圆
        for (size_t i=0; i< circles.size(); i++)
            if (Per(circles[i][0], circles[i][1], circles[i][2])<0.8
               || isCircle(circles[i][0], circles[i][1], circles[i][2])<0.95)
                flag[(int)i]=1;

        //对圆重叠进行判断
        while (1)
        {
            size_t Now;
            size_t i;
	    //找到未扫描过的圆，找到所有与其重叠（包括：两个圆不重叠，但有另一个圆与这两个圆同时重叠）的圆并标记
            for (i = 0; i < circles.size(); i++)
                if (flag[(int)i] == 0)
                {
                    Now = Floodfill(i);
                    break;
                }

            if (i >= circles.size()) break;
            //通过两种方式得到最优圆
            //1.有效颜色占比超过0.9的圆中最大的圆
            //2.没有有效颜色占比超过0.9的圆时采用有效颜色占比最高的圆
            double max = Per(circles[Now][0], circles[Now][1], circles[Now][2]);
            size_t best = Now;
            int maxR = -1;

            for (i = 0; i < circles.size(); i++)
            {
                if (flag[(int)i] == 2)
                {
                    flag[(int)i] = 1;
                    if ((circles[i][2]*1.1) < circles[Now][2])
                        continue;
                    double p = Per(circles[i][0], circles[i][1], circles[i][2]);
                    if (p > max)
                    {
                        best = i;
                        max = p;
                    }
                    if (p > 0.90  &&  (maxR == -1 || circles[i][2] > circles[(size_t)maxR][2]))
                        maxR = (int)i;
                }
            }
            if (maxR != -1) best = (size_t)maxR;
            flag[(int)best] = 3;
        }

        type Circle[10];
        int tot=0;
        for (size_t i = 0; i < circles.size(); i++)
        {
            if (flag[(int)i] != 3) continue;
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            //绘制圆心
            circle(rgbmat, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            //绘制圆轮廓
            circle(rgbmat, center, radius, Scalar(155, 50, 255), 3, 8, 0);
            if (tot < 3)
            {
                CvScalar mid = Color(circles[i][0], circles[i][1], circles[i][2]);	//得到圆颜色
                Circle[tot].RorG = (int)mid.val[0];
                Circle[tot].x = circles[i][0];
                Circle[tot].y = circles[i][1];
                Circle[tot].r = circles[i][2];
                //std::cout << (int)mid.val[0] << ' ' << (int)mid.val[1] << ' ' << (int)mid.val[2] << std::endl;
                Circle[tot].Dis = Dist(dep, circles[i][0], circles[i][1], circles[i][2]);	//得到圆距离
            }
            tot++;
            //std::cout << Dist(dep, circles[i][0], circles[i][1], circles[i][2]) << std::endl;
        }
        printf("tot = %d\n", tot);
        if (tot == 3)
        {
            sort(Circle, Circle+3, cmp);
            double argDis = (Circle[0].Dis + Circle[1].Dis + Circle[2].Dis)/3.0;
            if (sqr(argDis-Circle[0].Dis) + sqr(argDis-Circle[1].Dis) + sqr(argDis-Circle[2].Dis) < 750.0)
            {
                printf("%0.2lf\n", argDis);
                if (argDis < 1000.0)
                {
                    k = 25;
                    minDis = 2;
                    minR = 20;
                }
                if (argDis < 900.0)
                {
                    k = 25;
                    minDis = 2;
                    minR = 25;
                }
                if (argDis < 800.0)
                {
                    k = 27;
                    minDis = 3;
                    minR = 30;
                }
                if (argDis < 700.0)
                {
                    k = 28;
                    minDis = 3;
                    minR = 35;
                }
                if (argDis < 600.0)
                {
                    k = 28;
                    minDis = 4;
                    minR = 40;
                }
                if (argDis < 550.0)
                {
                    k = 28;
                    minDis = 5;
                    minR = 50;
                }
                if (argDis < 500.0)
                {
                    k = 30;
                    minDis = 10;
                    minR = 80;
                }
                if (argDis < 700.0)
                {
                    if (Circle[0].RorG == 250)
                    {
                        strcpy(send_buf, "00turnrrg");
                        printf("Right\n");
                    }
                    else if (Circle[1].RorG == 250)
                    {
                        strcpy(send_buf, "00turnrgr");
                        printf("Go\n");
                    }
                    else if (Circle[2].RorG == 250)
                    {
                        strcpy(send_buf, "00turngrr");
                        printf("Left\n");
                    }
                    else
                    {
                        strcpy(send_buf, "00turnrrr");
                        printf("Stop\n");
                    }

                    int re = send(sockfd, send_buf, strlen(send_buf), 0);   // 向服务器发送信息
                    if (re < 0)
                    {
                        printf("Send fail\n");
                    }

                    if (Circle[0].RorG == 250 || Circle[2].RorG == 250)
                    {
                        re = recv(sockfd, recv_buf, 10, 0);
                        if (re <= 0)
                        {
                            printf("Recv fail\n");
                        }
                        else
                        {
                            printf("Turn");
                            k = 25;
                            minR = 10;
                            minDis = 1;
                        }
                    }
                }
            }
        }

        cv::imshow("rgb", rgbmat);
        cv::imshow("hsv", hsv);

        //registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

        listener.release(frames);

    }

    dev->stop();
    dev->close();

    delete registration;

#endif

    std::cout << "Goodbye World!" << std::endl;
    return 0;
}
