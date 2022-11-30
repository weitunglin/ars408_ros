"""Decompress a compressed image into a raw image."""
import argparse
from rosbag import Bag
from tqdm import tqdm
from cv_bridge import CvBridge

def decompress(
    input_file: Bag,
    output_file: Bag,
    camera_info: list,
    compressed_camera: list,
    raw_camera: list,
) -> None:
    """
    Decompress a compressed image into a raw image.

    Args:
        input_file: the input bag to stream from
        output_file: the output bag to write data to
        camera_info: a list of info topics for each compressed image topic
        compressed_camera: a list of compressed image topics
        raw_camera: a list of raw image topics to publish

    Returns:
        None

    """
    # setup a dictionary mapping compressed topics to raw camera topics
    raw_camera = dict(zip(compressed_camera, raw_camera))
    # get the total number of output messages as total minus camera info topics
    total = input_file.get_message_count()
    bridge = CvBridge()
    # create a progress bar for iterating over the messages in the output bag
    with tqdm(total=total) as prog:
        # iterate over the messages in this input bag
        for topic, msg, time in input_file:
            # don't write the camera info to the bag
            if topic in camera_info:
                continue
            # if the topic is the compress image topic, decompress the image
            if topic in compressed_camera:
                # get the image from the compressed topic
                # img = get_camera_image(msg.data, dims[topic])
                img = bridge.compressed_imgmsg_to_cv2(msg)
                # create a raw image message and replace the message
                # msg = image_msg(img, time, dims[topic], 'rgb8')
                msg = bridge.cv2_to_imgmsg(img)
                # reset the topic
                topic = raw_camera[topic]
            # update the progress bar with a single iteration
            prog.update(1)
            # write the message
            output_file.write(topic, msg, time)


# ensure this script is running as the main entry point
if __name__ == '__main__':
    # create an argument parser to read arguments from the command line
    PARSER = argparse.ArgumentParser(description=__doc__)
    # add an argument for the output bag to create
    PARSER.add_argument('--output_bag', '-o',
        type=str,
        help='The path to an output bag file to write to.',
        required=True,
    )
    # add an argument for the input bag to clip
    PARSER.add_argument('--input_bag', '-i',
        type=str,
        help='The path to an input bag to clip.',
        required=True,
    )
    # add an argument for the camera info for the compressed image
    PARSER.add_argument('--camera_info', '-I',
        type=str,
        nargs='+',
        help='The topic to decompress into a raw image topic.',
        required=True,
    )
    # add an argument for the compressed camera image topic
    PARSER.add_argument('--compressed_camera', '-c',
        type=str,
        nargs='+',
        help='The topic to decompress into a raw image topic.',
        required=True,
    )
    # add an argument for the raw camera image topic
    PARSER.add_argument('--raw_camera', '-r',
        type=str,
        nargs='+',
        help='The name of the raw image topic to publish.',
        required=True,
    )
    try:
        # get the arguments from the argument parser
        ARGS = PARSER.parse_args()
        # open the input bag with an automatically closing context
        with Bag(ARGS.input_bag, 'r') as input_bag:
            # open the output bag in an automatically closing context
            with Bag(ARGS.output_bag, 'w') as output_bag:
                # decompress the topic in the input bag to the output bag
                decompress(
                    input_bag,
                    output_bag,
                    ARGS.camera_info,
                    ARGS.compressed_camera,
                    ARGS.raw_camera
                )
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [decompress.__name__]

"""
python3 decompress.py -i <in.bag> -o <out.bag> -c /rgb/front_center/compressed_image /rgb/front_left/compressed_image /rgb/front_right/compressed_image /rgb/rear_center/compressed_image /rgb/rear_left/compressed_image /rgb/rear_right/compressed_image /rgb/right_mirror/compressed_image /rgb/left_mirror/compressed_image /rgb/thermal/compressed_image -r /rgb/front_center/original_image /rgb/front_left/original_image /rgb/front_right/original_image /rgb/rear_center/original_image /rgb/rear_left/original_image /rgb/rear_right/original_image /rgb/right_mirror/original_image /rgb/left_mirror/original_image /rgb/thermal/original_image -I ""
"""
