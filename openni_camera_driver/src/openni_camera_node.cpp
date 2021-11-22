#include <cstdio>
// RCLCPP
#include <rclcpp/rclcpp.hpp>
// OpenNI packages
#include <OpenNI.h>

#define BOOST_NO_EXCEPTIONS

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/throw_exception.hpp>

void boost::throw_exception(std::exception const& e) {
    //do nothing
}

class CameraListener : public rclcpp::Node,
    public openni::VideoStream::NewFrameListener
{
private:
    cv::Mat last_img;
    openni::VideoFrameRef m_frame;
    std_msgs::msg::Header msg_img_header;
    sensor_msgs::msg::CompressedImage msg_compressed;
    sensor_msgs::msg::CameraInfo camera_info;
    std::mutex mtx;
   
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_image;
public:

    CameraListener(const std::string name, 
        const std::string topic_name,
        const std::string frame_id):
        Node(name+"_publisher")
    {
        msg_img_header.frame_id = frame_id;
        //
        msg_compressed.header.frame_id = frame_id;
        this->pub_image = pub_image;
        
    }

    void initialize(const std::string& topic_name, rclcpp::QoS& qos)
    {
        pub_image = create_publisher<sensor_msgs::msg::CompressedImage>(topic_name, qos);
    }

    cv::Mat getImage()
    {
        std::lock_guard<std::mutex> g(mtx);
        return last_img;
    }

    void onNewFrame(openni::VideoStream& stream)
    {
        stream.readFrame(&m_frame);
        // Process frame
        openni::DepthPixel* pDepth;
        
        

        using namespace openni;
        switch (m_frame.getVideoMode().getPixelFormat())
        {
        case PIXEL_FORMAT_DEPTH_1_MM:
        case PIXEL_FORMAT_DEPTH_100_UM:
        {
            pDepth = (DepthPixel*)m_frame.getData();
            auto start = std::chrono::high_resolution_clock::now();
            msg_compressed.header.stamp = now();
            const int width = m_frame.getWidth(); const int height = m_frame.getHeight();
            cv::Mat m_depth(height, width, CV_16UC1);
            m_depth.step = sizeof(unsigned char) * 2 * m_frame.getWidth();
            memcpy(m_depth.data, m_frame.getData(), m_frame.getDataSize());
            msg_compressed.format = "png";
            cv::imencode(".png", m_depth, msg_compressed.data);
            if (pub_image->get_subscription_count() > 0)
                pub_image->publish(msg_compressed);
            auto finish = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> g(mtx);
                last_img = m_depth;
            }
            std::chrono::duration<double> elapsed = finish - start;
            RCLCPP_INFO(get_logger(), "%d", elapsed);
            break;
        }
        case PIXEL_FORMAT_RGB888:
        {
            auto start = std::chrono::high_resolution_clock::now();
            openni::RGB888Pixel* pColor;
            pColor = (RGB888Pixel*)m_frame.getData();
            msg_compressed.header.stamp = now();
            const int width = m_frame.getWidth(); const int height = m_frame.getHeight();            
            cv::Mat m_rgb(height, width, CV_8UC3);
            memcpy(m_rgb.data, m_frame.getData(), (height * width * sizeof(uchar)*3));
            cv::cvtColor(m_rgb, m_rgb, cv::COLOR_BGR2RGB);
            msg_compressed.format = "jpg";
            cv::imencode(".jpg", m_rgb, msg_compressed.data);
            if (pub_image->get_subscription_count() > 0)
                pub_image->publish(msg_compressed);
            auto finish = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> g(mtx);
                last_img = m_rgb;
            }
            std::chrono::duration<double> elapsed = finish - start;
            RCLCPP_INFO(get_logger(), "%d", elapsed);
            break;
        }
        case PIXEL_FORMAT_YUV422:
        {
            openni::YUV422DoublePixel* pColor;
            const int width = m_frame.getWidth(); const int height = m_frame.getHeight();
            cv::Mat m_yuv422(height + height / 2, width, CV_8UC2);
            memcpy(m_yuv422.data, m_frame.getData(), (height + height) * width * sizeof(uchar));
            cv::Mat m_rgb(height, width, CV_8UC3);
            cv::cvtColor(m_yuv422, m_rgb, cv::COLOR_YUV2BGR_Y422);
            m_rgb = m_rgb(cv::Range(0, height), cv::Range(0, width));
            msg_compressed.header.stamp = now();
            msg_compressed.format = "png";
            cv::imencode(".png", m_rgb, msg_compressed.data);
            if (pub_image->get_subscription_count() > 0)
                pub_image->publish(msg_compressed);
            last_img = m_rgb;
            break;
        }
        default:
            std::cout << "Unknown format: " << m_frame.getVideoMode().getPixelFormat() << std::endl;
        }
    }

};

class OpenNiCameraNode : public rclcpp::Node,
    public openni::OpenNI::DeviceConnectedListener,
    public openni::OpenNI::DeviceDisconnectedListener,
    public openni::OpenNI::DeviceStateChangedListener    
{
private:
    openni::Device device;
    openni::VideoStream depth;
    openni::VideoStream color;
    openni::VideoFrameRef m_frame;
    //
    rclcpp::executors::MultiThreadedExecutor& executor;
    //
    rclcpp::TimerBase::SharedPtr pointcloud_timer;
    // Msg
    // Publishers
    //rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_color_image;
    //rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_depth_image;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl;
    // Frame listener
    std::shared_ptr<CameraListener> color_listener;
    std::shared_ptr<CameraListener> depth_listener;
public:
    OpenNiCameraNode(const std::string name, rclcpp::executors::MultiThreadedExecutor& executor) : Node(name), executor(executor)
    {
        
    }

    ~OpenNiCameraNode()
    {
        depth.destroy();
        color.destroy();
    }

    virtual void onDeviceConnected(const openni::DeviceInfo* pInfo)
    {
        RCLCPP_INFO(get_logger(), "Connected device: %s", pInfo->getUri());
    }

    virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo)
    {
        RCLCPP_INFO(get_logger(), "Disconnected device: %s", pInfo->getUri());
    }

    virtual void onDeviceStateChanged(const openni::DeviceInfo* pInfo, openni::DeviceState state)
    {
        RCLCPP_INFO(get_logger(), "Device \"%s\" error state changed to: %d\n", pInfo->getUri(), state);
    }

    int initialize()
    {        
        // Message headers
        // ROS 2 publishers
        size_t depth_ = 0;
        rmw_qos_history_policy_t history_policy_ = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        rmw_qos_reliability_policy_t relaibility_policy_ = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
                // The history policy determines how messages are saved until taken by
                // the reader.
                // KEEP_ALL saves all messages until they are taken.
                // KEEP_LAST enforces a limit on the number of messages that are saved,
                // specified by the "depth" parameter.
                history_policy_,
                // Depth represents how many messages to store in history when the
                // history policy is KEEP_LAST.
                depth_
                ));
        qos.reliability(relaibility_policy_);
        color_listener = std::make_shared<CameraListener>("color_camera_publisher", "color", "openni_camera");
        color_listener->initialize("/color/image", qos);
        executor.add_node(color_listener);
        depth_listener = std::make_shared<CameraListener>("depth_camera_publisher", "depth", "openni_camera");
        //pub_depth_image = depth_listener->create_publisher<sensor_msgs::msg::CompressedImage>("depth/image", qos);
        pub_pcl = depth_listener->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", qos);
        depth_listener->initialize("depth/image", qos);
        executor.add_node(depth_listener);
        RCLCPP_INFO(get_logger(), "Initialized ROS2, starting camera");
        // Initialize OpenNI context
        using namespace openni;
        Status rc = openni::OpenNI::initialize();
        if (rc != STATUS_OK)
        {
            RCLCPP_FATAL(get_logger(), "Unable to initialize node");
            return 1;
        }
        // Setup basic listeners
        OpenNI::addDeviceConnectedListener(this);
        OpenNI::addDeviceConnectedListener(this);
        OpenNI::addDeviceDisconnectedListener(this);
        // Setup device                
        // Open device
        rc = device.open(ANY_DEVICE);
        if (rc != STATUS_OK)
        {
            RCLCPP_FATAL(get_logger(), "Unable to open any OpenNI device");
            return 2;
        }
        // Resolution Color
        const openni::SensorInfo* sinfo = device.getSensorInfo(openni::SENSOR_COLOR); // select index=4 640x480, 30 fps, 1mm
        const openni::Array<openni::VideoMode>& modesColor = sinfo->getSupportedVideoModes();
        openni::VideoMode mode = modesColor[0];
        unsigned int resX = 1280;
        unsigned int resY = 720;
        unsigned int targetFps = 30;
        for (int i = 0; i < modesColor.getSize(); i++) {
            printf("%i: %ix%i, %i fps, %i format\n", i, modesColor[i].getResolutionX(), modesColor[i].getResolutionY(),
                modesColor[i].getFps(), modesColor[i].getPixelFormat()); //PIXEL_FORMAT_DEPTH_1_MM = 100, PIXEL_FORMAT_DEPTH_100_UM
            if (modesColor[i].getResolutionX() == resX && modesColor[i].getResolutionY() == resY && modesColor[i].getFps() == targetFps)
            {
                mode = modesColor[i];
            }
        }
        mode = modesColor[4];
        std::cout << mode.getResolutionX() << " " << mode.getResolutionY() << std::endl;
        if (rc != STATUS_OK)
        {
            RCLCPP_FATAL(get_logger(), "Unable to open any OpenNI device");
            return 2;
        }
        // Attempt to start visual stream
        if (device.getSensorInfo(SENSOR_COLOR) != NULL)
        {
            rc = color.create(device, SENSOR_COLOR);
            if (rc != STATUS_OK)
            {
                printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
            }
            color.setVideoMode(mode);
        }
        rc = color.start();
        std::cout << "FOV:  Horz: " << color.getHorizontalFieldOfView() << ", Vert: " << color.getVerticalFieldOfView() << std::endl;
        if (rc != STATUS_OK)
        {
            printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
        }
        color.addNewFrameListener(color_listener.get());
        // Attempt to start depth stream
        sinfo = device.getSensorInfo(openni::SENSOR_DEPTH); // select index=4 640x480, 30 fps, 1mm
        const openni::Array<openni::VideoMode>& modesDepth = sinfo->getSupportedVideoModes();
        mode = modesDepth[0];
        for (int i = 0; i < modesDepth.getSize(); i++) {
            printf("%i: %ix%i, %i fps, %i format\n", i, modesDepth[i].getResolutionX(), modesDepth[i].getResolutionY(),
                modesDepth[i].getFps(), modesDepth[i].getPixelFormat()); //PIXEL_FORMAT_DEPTH_1_MM = 100, PIXEL_FORMAT_DEPTH_100_UM
            if (modesDepth[i].getResolutionX() == resX && modesDepth[i].getResolutionY() == resY && modesDepth[i].getFps() == targetFps)
            {
                mode = modesDepth[i];
            }
        }
        mode = modesDepth[4];
        std::cout << mode.getResolutionX() << " " << mode.getResolutionY() << std::endl;
        if (rc != STATUS_OK)
        {
            RCLCPP_FATAL(get_logger(), "Unable to open any OpenNI device");
            return 2;
        }
        // TODO separate publishers
        if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
        {
            rc = depth.create(device, SENSOR_DEPTH);
            if (rc != STATUS_OK)
            {
                printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
            }
            depth.setVideoMode(mode);
        }
        rc = depth.start();
        if (rc != STATUS_OK)
        {
            printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
        }
        std::cout << "Depth FOV:  Horz: " << depth.getHorizontalFieldOfView() << ", Vert: " << depth.getVerticalFieldOfView() << std::endl;
        depth.addNewFrameListener(depth_listener.get());
        // Timer
        using namespace std::chrono_literals;
        pointcloud_timer = create_wall_timer(100ms, std::bind(&OpenNiCameraNode::pointcloud_wall_timer, this));
        // Everything is alright
        
        // ROS 2 subscribers
        RCLCPP_INFO(get_logger(), "Initialized node");
        return 0;
    }

    void pointcloud_wall_timer()
    {
        auto color_image = color_listener->getImage();
        auto depth_image = depth_listener->getImage();
        // Get parameters
        // TODO: set parameters from file
        //const double fx_d = 958.188;
        const double fx_d = 958.188;
        const double fy_d = 4095.017;
        const double cx_d = 320.0;
        const double cy_d = 240.0;
        // 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        ptCloud->resize(color_image.cols * color_image.rows);
        int r = color_image.rows;
        depth_image.forEach<uint16_t>([&ptCloud, &color_image, r, fx_d, fy_d, cx_d, cy_d](uint16_t &p, const int * position) {
            float z = static_cast<float>(p) * 0.001;
            int x = position[0]; int y = position[1];
            (*ptCloud)[x + y * r].x = z;            
            (*ptCloud)[x + y * r].y = - z * (cy_d - y) / fy_d;
            (*ptCloud)[x + y * r].z = - z * (x - cx_d) / fx_d;
            // Color
            (*ptCloud)[x + y * r].r = color_image.at<cv::Vec3b>(x, y)[0];
            (*ptCloud)[x + y * r].g = color_image.at<cv::Vec3b>(x, y)[1];
            (*ptCloud)[x + y * r].b = color_image.at<cv::Vec3b>(x, y)[2];
            
        });
        ptCloud->width = (int)color_image.cols;
        ptCloud->height = (int)color_image.rows;
        pcl::PCLPointCloud2 msg;
        pcl::toPCLPointCloud2<pcl::PointXYZRGB>(*ptCloud, msg);
        sensor_msgs::msg::PointCloud2 pt2;
        //        
        pcl_conversions::moveFromPCL(msg, pt2);
        pt2.header.frame_id = "openni_camera";

        //
        pub_pcl->publish(pt2);
        //
        RCLCPP_INFO(get_logger(), "Depth merging");
    }
    

    void disconnectDevice()
    {
        // Remove frame listeners
        color.removeNewFrameListener(color_listener.get());
        depth.removeNewFrameListener(depth_listener.get());

        // Stop visual
        color.stop();
        color.destroy();
        // Stop depth
        depth.stop();
        depth.destroy();
        device.close();
        RCLCPP_INFO(get_logger(), "Successfully disconnected OpenNI devices");
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    // Initialize ROS2 node
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<OpenNiCameraNode> camera_node = std::make_shared<OpenNiCameraNode>("openni_camera_node", executor);
    //
    
    
    int status = camera_node->initialize();
    if (status == 0)
    {
        RCLCPP_INFO(camera_node->get_logger(), "Successfully connected OpenNI devices");
        executor.add_node(camera_node);
        executor.spin();
        camera_node->disconnectDevice();
        rclcpp::shutdown();

        return 0;
    }
    return status;
    
}
