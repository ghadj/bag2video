#!/usr/bin/env python3

from __future__ import division
import rosbag
import rospy
import numpy as np
import sys
import os
import cv2
import glob
from itertools import repeat
import argparse

CAMERA_INFO_TOPIC = '/kinect/rgb/camera_info'
RGB_TOPIC = '/kinect/rgb/image_raw/compressed'

# try to find cv_bridge:
try:
    from cv_bridge import CvBridge
except ImportError:
    # assume we are on an older ROS version, and try loading the dummy manifest
    # to see if that fixes the import error
    try:
        import roslib
        roslib.load_manifest("bag2video")
        from cv_bridge import CvBridge
    except:
        print("Could not find ROS package: cv_bridge")
        print("If ROS version is pre-Groovy, try putting this package in ROS_PACKAGE_PATH")
        sys.exit(1)


def get_info(bag, topic_info=None, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize)):
    size = (0, 0)
    times = []

    # read the first message to get the image size
    msg = next(bag.read_messages(topics=topic_info))[1]
    size = (msg.width, msg.height)

    # now read the rest of the messages for the rates
    iterator = bag.read_messages(
        topics=RGB_TOPIC, start_time=start_time, end_time=stop_time)  # , raw=True)
    for _, msg, _ in iterator:
        time = msg.header.stamp
        times.append(time.to_sec())
        # size = (msg.width, msg.height)
    diffs = 1/np.diff(times)
    return np.median(diffs), min(diffs), max(diffs), size, times


def calc_n_frames(times, precision=10):
    # the smallest interval should be one frame, larger intervals more
    intervals = np.diff(times)
    return np.int64(np.round(precision*intervals/min(intervals)))


def write_frames(bag, writer, total, topic_rgb_name=None, nframes=repeat(1), start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize), viz=False, encoding='bgr8'):
    bridge = CvBridge()
    if viz:
        cv2.namedWindow('win')
    count = 1

    iterator = bag.read_messages(
        topics=topic_rgb_name, start_time=start_time, end_time=stop_time)
    for (topic, msg, time), reps in zip(iterator, nframes):
        print('\rWriting frame %s of %s at time %s' % (count, total, time),)

        # https://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber#CA-f9340566f18b7f023b315a570812267b4ce36abf_28
        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # img = np.asarray(bridge.imgmsg_to_cv2(msg, 'bgr8'))
        for _ in range(reps):
            writer.write(img)
        imshow('win', img)
        count += 1


def imshow(win, img):
    cv2.imshow(win, img)
    cv2.waitKey(1)


def noshow(win, img):
    pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Extract and encode video from bag files.')
    parser.add_argument('--outfile', '-o', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--precision', '-p', action='store', default=10, type=int,
                        help='Precision of variable framerate interpolation. Higher numbers\
                        match the actual framerater better, but result in larger files and slower conversion times.')
    parser.add_argument('--viz', '-v', action='store_true',
                        help='Display frames in a GUI window.')
    parser.add_argument('--start', '-s', action='store', default=0, type=float,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=sys.maxsize, type=float,
                        help='Rostime representing where to stop in the bag.')
    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='bgr8',
                        help='Encoding of the deserialized image.')

    parser.add_argument('--topic_rgb', '-trgb',
                        action='store', default=RGB_TOPIC)
    parser.add_argument('--topic_info', '-ti',
                        action='store', default=CAMERA_INFO_TOPIC)
    parser.add_argument('bagfile')

    args = parser.parse_args()

    if not args.viz:
        imshow = noshow

    start_time = rospy.Time(args.start)
    end_time = rospy.Time(args.end)

    for bagfile in glob.glob(args.bagfile):
        print(bagfile)

        outfile = args.outfile
        if not outfile:
            outfile = os.path.join(
                *os.path.split(bagfile)[-1].split('.')[:-1]) + '.avi'

        bag = rosbag.Bag(bagfile, 'r')

        print('Calculating video properties')
        rate, minrate, maxrate, size, times = get_info(
            bag, args.topic_info, start_time=start_time, stop_time=end_time)
        nframes = calc_n_frames(times, args.precision)
        # writer = cv2.VideoWriter(outfile, cv2.cv.CV_FOURCC(*'DIVX'), rate, size)
        writer = cv2.VideoWriter(outfile, cv2.VideoWriter_fourcc(
            *'DIVX'), np.ceil(maxrate*args.precision), size)

        print('Writing video')
        write_frames(bag, writer, len(times), topic_rgb_name=args.topic_rgb, nframes=nframes,
                     start_time=start_time, stop_time=end_time, encoding=args.encoding)
        writer.release()
