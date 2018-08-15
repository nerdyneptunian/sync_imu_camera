
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <SwarmFLIRWrapper/swarmimagehandler.h>
#include <SwarmFLIRWrapper/flircamera.h>

using namespace std;

class RosImageHandler : public SwarmImageHandler
{
public:
    RosImageHandler(Spinnaker::CameraPtr &cam, ros::Publisher &imagePublisher) : SwarmImageHandler(cam), publisher_(imagePublisher)
    {
        buffer_.resize(0);
        publishTime_ = ros::Time::now();
    }

    virtual void OnImageEvent(Spinnaker::ImagePtr image) override
    {
        if (image->IsIncomplete())
        {
            std::cout << "Image incomplete with image status " << image->GetImageStatus() << "..." << endl << endl;
        }
        else
        {
            auto currentTime = ros::Time::now();

            if((currentTime - publishTime_).toSec() > 1/maxPublishFPS_)
            {
                publishTime_ = currentTime;
                cv_bridge::CvImage cv_image;
                cv_image.header.stamp = currentTime;

                int width = image->GetWidth();
                int height = image->GetHeight();
                cv::Mat cvIm(height, width, CV_8UC1);
                // Print image information
                //memcpy(buffer_.getWritePtr(), image->GetData(), buffer_.imageSize());
                //buffer_.finishWrite();
                memcpy(cvIm.data, image->GetData(), width*height);
                cv_image.image = cvIm;
                cv_image.encoding = "mono8";
                cv_image.header.frame_id = imageCount_;
                //if(!(imageCount_ & 0x3F))
                //    ROS_INFO("Publishing image %i", imageCount_);
                publisher_.publish(cv_image.toImageMsg());
                imageCount_++;
                //ROS_INFO("%i", imageCount_);
            }
        }
    }

private:
    ros::Publisher publisher_;
    double maxPublishFPS_ = 80;
    ros::Time publishTime_;
};

int runCameraCapture();
int main(int argc, char **argv)
{
    ros::init(argc, argv, "flir_system");
    ros::start();

    int ret = runCameraCapture();

    ros::shutdown();

    return ret;
}

int runCameraCapture()
{
    FlirCamera cam_;
    cam_.open();
    if(!cam_.isReady())
    {
        ROS_INFO("Could not open camera");
        return 1;
    }

    cam_.setTriggerMode(false);
    //cam_.setTriggerMode(true);

    ros::NodeHandle nodeHandler;
    ros::Publisher imagePublisher = nodeHandler.advertise<sensor_msgs::Image>("/camera/image_raw", 1);

    RosImageHandler imHandler(cam_.getCam(), imagePublisher);
    ros::spinOnce();

    cam_.startAcquisition();

    ROS_INFO("Starting camera server");

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Stopping camera...");
    cam_.endAcquisition();

    return 0;
}
