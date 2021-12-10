from cv_bridge import CvBridge
import argparse
import cv2
import os
import pandas as pd
import rosbag
import shutil


def yesno(question):
    answer = input(question + ' (y/n): ')
    if answer in ['y', 'Y', 'yes', 'Yes', 'YES']:
        return 1
    else:
        return 0


def main():
    parser = argparse.ArgumentParser(description="Extract images and yaw value from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("csv_file", help="Output csv file")

    args = parser.parse_args()

    if not os.path.exists(args.bag_file):
        print("ERROR: Invalid bag file")
        print("Exiting")
        exit(1)
    if os.path.exists(args.output_dir):
        if len(os.listdir(args.output_dir)) != 0:
            print("WARNING: Output dir is not empty")
            ans = yesno("Do you want to replace the contents of '%s/' ?" % args.output_dir)
            if not ans:
                print("Exiting")
                exit(1)
            shutil.rmtree(args.output_dir, ignore_errors=False, onerror=None)
            os.mkdir(args.output_dir)
    else:
        print("Creating new output directory")
        os.mkdir(args.output_dir)

    image_topic = "/naiad/simulation/camera/image_color"
    odometry_topic = "/naiad/simulation/odometry"

    """Extract a folder of images from a rosbag.
    """
    print("Extract images from %s on topic %s into %s" % (args.bag_file,
        image_topic, args.output_dir))

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), cv_img)
        count += 1

    print("Wrote %i images" % count)
    bag.close()

    """Extract yaw angles from a rosbag.
    """
    print("Extract yaw angles from %s on topic %s into %s" % (args.bag_file,
        odometry_topic, args.csv_file))
    # The bag file should be in the same directory as your terminal
    bag = rosbag.Bag(args.bag_file)
    column_names = ['time', 'yaw']
    df = pd.DataFrame(columns=column_names)

    for topic, msg, t in bag.read_messages(topics=odometry_topic):
        z = msg.pose.pose.orientation.z
        time = str(msg.header.stamp.secs) + '.' + str(msg.header.stamp.nsecs)

        df = df.append(
            {'time': time,
             'yaw': z},
            ignore_index=True)

    df.to_csv(args.csv_file)

    return


if __name__ == '__main__':
    main()
