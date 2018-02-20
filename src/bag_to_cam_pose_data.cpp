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

#include <fstream>

#include "libnpy.hpp"

#include <tf/transform_datatypes.h>


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

// Global variables for storing the necessary data
std::vector<unsigned long> img_shape;
std::vector<uint8_t> img_data;
std::vector<unsigned long> pose_shape;
std::vector<double> pose_data;

// Callback for synchronized messages
void callback(
    const sensor_msgs::Image::ConstPtr& image,
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
    // First deal with the image data.
    if (img_shape.size() == 0)
    {
        img_shape.push_back(0);
        img_shape.push_back((unsigned long) image->height);
        img_shape.push_back((unsigned long) image->width);
        img_shape.push_back((unsigned long) 3);
    }
    // Increase number of images in shape
    img_shape[0]++;
    // Add image to data array
    img_data.insert(std::end(img_data), std::begin(image->data), std::end(image->data));

    // Now deal with the pose data.
    if (pose_shape.size() == 0)
    {
        pose_shape.push_back(0);
        // 3 for the position and another 3 for the rotation (xyzrpy)
        pose_shape.push_back(6);
    }
    pose_shape[0]++;

    tf::Quaternion q;
    tf::quaternionMsgToTF(pose->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose_data.push_back(pose->pose.pose.position.x);
    pose_data.push_back(pose->pose.pose.position.y);
    pose_data.push_back(pose->pose.pose.position.z);
    pose_data.push_back(roll);
    pose_data.push_back(pitch);
    pose_data.push_back(yaw);
}
void write_data(const std::string& output_image_path, const std::string& output_pose_path, int& output_cnt)
{ 
    if (pose_data.size() == 0)
    {
        return;
    }
    // Output the npy files
    npy::SaveArrayAsNumpy(output_image_path + std::to_string(output_cnt), false, img_shape.size(), &img_shape[0], img_data);
    npy::SaveArrayAsNumpy(output_pose_path + std::to_string(output_cnt), false, pose_shape.size(), &pose_shape[0], pose_data);
    output_cnt++;
    img_shape.clear();
    img_data.clear();
    pose_shape.clear();
    pose_data.clear();
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
    ValueArg<std::string> output_image_file_arg(
        "x",
        "output_image_file",
        "The path to to the output file containing the images.",
        true,
        "",
        "string"
    );
    ValueArg<std::string> output_pose_file_arg(
        "y",
        "output_pose_file",
        "The path to to the output file containing the poses.",
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
    ValueArg<double> split_after_arg(
        "s",
        "split_after",
        "The number of seconds after which a new npy file is created.",
        false,
        86400.0,
        "double"
    );
    cmdline.add(input_bag_arg);
    cmdline.add(output_image_file_arg);
    cmdline.add(output_pose_file_arg);
    cmdline.add(image_topic_arg);
    cmdline.add(pose_topic_arg);
    cmdline.add(split_after_arg);
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

    ros::Duration recording_duration(split_after_arg.getValue(), 0.0);
    //std::cout << "split after: " << split_after_arg.getValue() << " ";
    ros::Time start, end;
    bool is_recording = false;
    int output_cnt = 0;

    bool first_time = true;

    // Load all messages into our stereo dataset
    for (const rosbag::MessageInstance& m : view)
    {
        ros::Time t = m.getTime();

        // If time is outside of window.
        if (!first_time and !(start <= t and t < end))
        {
            // std::cout << " -- dumping both files (" << output_image_file_arg.getValue() + std::to_string(output_cnt) << ")";
            write_data(output_image_file_arg.getValue(), output_pose_file_arg.getValue(), output_cnt);
            is_recording = false;
        }
        
        // If we are currently not recording initialize the window.
        if (not is_recording)
        {
            start = t;
            end = t + recording_duration;
            is_recording = true;
            first_time = false;
            // std::cout << "opening window from " << start << " to " << end;
        }

        if (m.getTopic() == image_topic || ("/" + m.getTopic() == image_topic))
        {
            // std::cout << " -- reading image";
            sensor_msgs::Image::ConstPtr image = m.instantiate<sensor_msgs::Image>();
            if (image != NULL)
            {
                image_sub.newMessage(image);
            }
        }

        if (m.getTopic() == pose_topic || ("/" + m.getTopic() == pose_topic))
        {
            // std::cout << " -- reading pose";
            geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
            if (pose != NULL)
            {
                pose_sub.newMessage(pose);
            }
        }
        // std::cout << std::endl;
    }
    bag.close();

    // Output the rest.
    write_data(output_image_file_arg.getValue(), output_pose_file_arg.getValue(), output_cnt);
}

