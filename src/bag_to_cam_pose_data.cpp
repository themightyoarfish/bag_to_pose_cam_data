#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_datatypes.h>

#include <tclap/CmdLine.h>
#include "libnpy.hpp"

#include <fstream>

#include <boost/filesystem.hpp>


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
bool create_single_files = false;
std::string output_image_path_string;
std::string output_pose_path_string;
bool export_as_float = false;
int output_cnt;


void write_data(const std::string& output_image_path, const std::string& output_pose_path, int& output_cnt)
{
    if (pose_data.size() == 0)
    {
        return;
    }

    std::stringstream ss_img;
    ss_img << output_image_path << std::setfill('0') << std::setw(6) << output_cnt << ".npy";

    std::stringstream ss_pose;
    ss_pose << output_pose_path << std::setfill('0') << std::setw(6) << output_cnt << ".npy";


    // Output the npy files
    if (export_as_float)
    {
        std::vector<float> float_image(std::begin(img_data), std::end(img_data));
        npy::SaveArrayAsNumpy(ss_img.str(), false, img_shape.size(), &img_shape[0], float_image);
    }
    else
    {
        npy::SaveArrayAsNumpy(ss_img.str(), false, img_shape.size(), &img_shape[0], img_data);
    }
    npy::SaveArrayAsNumpy(ss_pose.str(), false, pose_shape.size(), &pose_shape[0], pose_data);
    output_cnt++;
    img_shape.clear();
    img_data.clear();
    pose_shape.clear();
    pose_data.clear();
}



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

    if (create_single_files)
    {
        write_data(output_image_path_string, output_pose_path_string, output_cnt);
    }
}


int main(int argc, char const *argv[])
{

    ros::Time::init();

    using TCLAP::CmdLine;
    using TCLAP::ValueArg;
    using TCLAP::ValuesConstraint;
    using TCLAP::SwitchArg;

    // Namespace alias
    namespace bfs = boost::filesystem;

    CmdLine cmdline("Bag to cam pose extractor");

    ValueArg<std::string> input_bag_arg(
        "b",
        "input_bag",
        "The path to the input bagfile.",
        true,
        "",
        "string"
    );

    ValueArg<std::string> output_directory_arg(
        "d",
        "output_directory",
        "The path to where the images and poses should be put. A directory for the particular input will be created.",
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
    SwitchArg create_single_files_arg(
        "P",
        "create_single_files",
        "A switch that will overwrite the split_after arg and output all data points in single files."
    );
    SwitchArg export_as_float_arg(
        "x",
        "export_as_float",
        "If present image values will be exported as float isntead of uint8_t. Be aware of a quadrupled size tho."
    );

    cmdline.add(input_bag_arg);
    cmdline.add(output_directory_arg);
    cmdline.add(image_topic_arg);
    cmdline.add(pose_topic_arg);
    cmdline.add(split_after_arg);
    cmdline.add(create_single_files_arg);
    cmdline.add(export_as_float_arg);
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

    bfs::path output_directory(output_directory_arg.getValue());
    output_directory /= bfs::path(input_bag_arg.getValue()).stem();

    // Create directory (if not existent) with the same name as the bagfile name
    if (not bfs::exists(output_directory))
    {
        bfs::create_directory(output_directory);
    }
    else
    {
        if (not bfs::is_directory(output_directory))
        {
            throw std::runtime_error("Output folder cannot be created");
        }
    }

    // Create a directory called images if not existent
    bfs::path output_image_path(output_directory / bfs::path("images"));
    if (not bfs::exists(output_image_path))
    {
        bfs::create_directory(output_image_path);
    }
    else
    {
        if (not bfs::is_directory(output_image_path))
        {
            throw std::runtime_error("Output image folder cannot be created");
        }
    }
    ROS_INFO_STREAM("Writing images to directory: " << output_image_path);
    // Add filename stem to path
    output_image_path /= bfs::path("image");

    // Create a directory called poses if not existent
    bfs::path output_pose_path(output_directory / bfs::path("poses"));
    if (not bfs::exists(output_pose_path))
    {
        bfs::create_directory(output_pose_path);
    }
    else
    {
        if (not bfs::is_directory(output_pose_path))
        {
            throw std::runtime_error("Output poses folder cannot be created");
        }
    }
    ROS_INFO_STREAM("Writing poses to directory: " << output_pose_path);
    // Add filename stem to path
    output_pose_path /= bfs::path("pose");

    output_image_path_string = output_image_path.string();
    output_pose_path_string  = output_pose_path.string();

    create_single_files = create_single_files_arg.getValue();
    export_as_float = export_as_float_arg.getValue();

    ros::Duration recording_duration(split_after_arg.getValue(), 0.0);
    ros::Time start, end;
    bool is_recording = false;
    output_cnt = 0;

    bool first_time = true;
    // Load all messages into our stereo dataset
    for (const rosbag::MessageInstance& m : view)
    {
        if (not create_single_files)
        {
            ros::Time t = m.getTime();

            // If time is outside of window.
            if (!first_time and !(start <= t and t < end))
            {
                write_data(output_image_path_string, output_pose_path_string, output_cnt);
                is_recording = false;
            }

            // If we are currently not recording initialize the window.
            if (not is_recording)
            {
                start = t;
                end = t + recording_duration;
                is_recording = true;
                first_time = false;
            }
        }

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

    // Output the rest.
    if (not create_single_files)
    {
        write_data(output_image_path_string, output_pose_path_string, output_cnt);
    }
}
