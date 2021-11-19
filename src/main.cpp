#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>

class SLAMNode: public rclcpp::Node
{
public:
    SLAMNode(): rclcpp::Node("test"), sync_policy_(10),
    image_sync_(sync_policy_) {
        auto subscribe_image_topic = [this](ImageSubscriberFilter &sub, std::string topic, const std::string = "raw") {
        sub.subscribe(this, topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    };
        subscribe_image_topic(sub_image_, "image");
        subscribe_image_topic(sub_depth_, "depth");
        image_sync_.connectInput(sub_image_, sub_depth_);
        image_sync_.registerCallback(
            std::bind(&SLAMNode::rgbd_callback, this, std::placeholders::_1, std::placeholders::_2));
    };
    ~SLAMNode() {};

    void rgbd_callback(const sensor_msgs::msg::Image::ConstSharedPtr msgRGB,
                       const sensor_msgs::msg::Image::ConstSharedPtr msgD) {
        std::cout << "Callback " << std::endl;
    }

private:
    typedef message_filters::Subscriber<sensor_msgs::msg::Image> ImageSubscriberFilter;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_policy;
    sync_policy sync_policy_;
    message_filters::Synchronizer<sync_policy> image_sync_;
    ImageSubscriberFilter sub_image_, sub_depth_, sub_right_image_, sub_mono_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<SLAMNode> node = std::make_shared<SLAMNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
 
    return EXIT_SUCCESS;
}

