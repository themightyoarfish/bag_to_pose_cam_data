#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <tclap/CmdLine.h>


// A struct to hold the synchronized camera data
// Struct to store stereo data
class CamPose
{

public:
    sensor_msgs::Image::ConstPtr image;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose;

    CamPose(
        const sensor_msgs::Image::ConstPtr &image,
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose
    ) :
        image(image),
        pose(pose)
    {}
};

std::vector<CamPose> cam_poses;

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(const boost::shared_ptr<M const> &msg)
    {
        this->signalMessage(msg);
    }
};

// Callback for synchronized messages
void callback(
    const sensor_msgs::Image::ConstPtr& image,
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
    ROS_INFO_STREAM(image->header.stamp << " : " << pose->header.stamp);
    CamPose cp(image, pose);
    cam_poses.push_back(cp);
}


int main(int argc, char const *argv[])
{
    ros::Time::init();

    using TCLAP::CmdLine;
    using TCLAP::ValueArg;
    using TCLAP::ValuesConstraint;
    using TCLAP::SwitchArg;

    CmdLine cmdline("Bag to cam pose extractor");

    ValueArg<std::string> input_bag_arg(
        "b",
        "input_bag",
        "The path to the input bagfile.",
        true,
        "",
        "string"
    );
    ValueArg<std::string> output_file_arg(
        "o",
        "output_file",
        "The path to to the output file.",
        true,
        "",
        "string"
    );
    ValueArg<std::string> image_topic_arg(
        "i",
        "image_topic",
        "The image topic.",
        false,
        "/usb_cam_node/image_raw",
        "string"
    );
    ValueArg<std::string> pose_topic_arg(
        "p",
        "pose_topic",
        "The Pose topic.",
        false,
        "/robot_pose_ekf/odom_combined",
        "string"
    );
    cmdline.add(input_bag_arg);
    cmdline.add(output_file_arg);
    cmdline.add(image_topic_arg);
    cmdline.add(pose_topic_arg);
    cmdline.parse(argc, argv);

    rosbag::Bag bag;
    bag.open(input_bag_arg.getValue(), rosbag::bagmode::Read);

    // Image topics to load
    std::vector<std::string> topics;
    topics.push_back(image_topic_arg.getValue());
    topics.push_back(pose_topic_arg.getValue());

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Set up fake subscribers to capture images
    BagSubscriber<sensor_msgs::Image> image_sub;
    BagSubscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseWithCovarianceStamped> camPosePolicy;
    message_filters::Synchronizer<camPosePolicy> sync(camPosePolicy(10), image_sub, pose_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    const std::string image_topic = image_topic_arg.getValue();
    const std::string pose_topic  = pose_topic_arg.getValue();

    // Load all messages into our stereo dataset
    for (const rosbag::MessageInstance& m : view)
    {
        if (m.getTopic() == image_topic || ("/" + m.getTopic() == image_topic))
        {
            sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
            if (image != NULL)
            {
                image_sub.newMessage(image);
            }
        }

        if (m.getTopic() == pose_topic || ("/" + m.getTopic() == pose_topic))
        {
            geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
            if (pose != NULL)
            {
                pose_sub.newMessage(pose);
            }
        }
    }
    bag.close();
}
