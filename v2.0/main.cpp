#include "kinect.h"
#include "pioneer.h"

int main(int argc , char** argv)
{
    /******************************************************************************/
    //launch kinect

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    std::string serial = "";

    bool enable_rgb = true;
    bool enable_depth = true;

    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    pipeline = new libfreenect2::CpuPacketPipeline();

    dev = freenect2.openDevice(serial , pipeline);

    int types = 0;
    if (enable_rgb)
    {
        types |= libfreenect2::Frame::Color;
    }
    if (enable_depth)
    {
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    }
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    if (enable_rgb && enable_depth)
    {
        if (!dev->start())
        {
            //return -1;
        }
    }
    else
    {
        if (!dev->startStreams(enable_rgb , enable_depth))
        {
            //return -1;
        }
    }

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Registration *registration = new libfreenect2::Registration(dev->getIrCameraParams() , dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512 , 424 , 4) , registered(512 , 424 , 4);



    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_img(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl_img->width = 512;
    pcl_img->height = 424;
    pcl_img->is_dense = false;
    pcl_img->resize(512 * 424);

    std::string file_No;
    std::string path("/home/csq/testPCD/testPCD");
    std::string ext(".pcd");
    std::string name("");
    int No = 0;

    pcl::visualization::CloudViewer pcl_viewer ("Cloud");

    //main loop

    while (1)
    {
        if (!listener.waitForNewFrame(frames , 3 * 1000))
        {
            std::cout << "timeout!" << std::endl;
        }
        else
        {
            libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
            //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

            registration->apply(rgb , depth , &undistorted , &registered);

            for(int i = 0;i < 424;++i)
            {
                for(int j = 0;j < 512;++j)
                {
                    registration->getPointXYZRGB(&undistorted , &registered , i , j , pcl_img->points[i * 512 + j].x ,
                    pcl_img->points[i * 512 + j].y , pcl_img->points[i * 512 + j].z , pcl_img->points[i * 512 + j].rgb);
                }
            }

            std::stringstream ss;
            ss << No;
            ss >> file_No;
            name = path + file_No + ext;

            pcl::io::savePCDFileASCII(name , *pcl_img);

            ++No;
            name.clear();

            usleep(100000);

            pcl_viewer.showCloud(pcl_img);

            listener.release(frames);

            //break;
        }
    }

    dev->stop();
    dev->close();

//    robot.waitForRunExit();

//    Aria::exit(0);

    return 0;
}
