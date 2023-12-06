"""Decompress a compressed image into a raw image."""
import argparse
import numpy as np
from rosbag import Bag
from tqdm import tqdm
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, PointField

def decompress(
    input_file: Bag,
    output_file: Bag,
    camera_info: list,
    compressed_camera: list,
    raw_camera: list,
    DECODE_POINTCLOUD: bool = False
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

            if DECODE_POINTCLOUD and "decoded_messages" in topic:
                pc_msg = PointCloud2() # ploint cloud message
                pc_msg.header = msg.header

                pc_msg.height = 1
                pc_msg.width = len(msg.rps)

                # pc_msg.fields = [
                #         PointField('x', 0, PointField.FLOAT32, 1),
                #         PointField('y', 4, PointField.FLOAT32, 1),
                #         PointField('z', 8, PointField.FLOAT32, 1),
                #         PointField('dyn_prop', 12, PointField.INT8, 1),
                #         PointField('id', 13, PointField.INT16, 1),
                #         PointField('rcs', 15, PointField.FLOAT32, 1),
                #         PointField('vx', 19, PointField.FLOAT32, 1),
                #         PointField('vy', 23, PointField.FLOAT32, 1),
                #         PointField('vx_comp', 27, PointField.FLOAT32, 1),
                #         PointField('vy_comp', 31, PointField.FLOAT32, 1),
                #         PointField('is_quality_valid', 32, PointField.INT8, 1),
                #         PointField('ambig_state', 33, PointField.INT8, 1),
                #         PointField('x_rms', 34, PointField.INT8, 1),
                #         PointField('y_rms', 35, PointField.INT8, 1),
                #         PointField('invalid_state', 36, PointField.INT8, 1),
                #         PointField('pdf0', 37, PointField.INT8, 1),
                #         PointField('vx_rms', 38, PointField.INT8, 1),
                #         PointField('vy_rms', 39, PointField.INT8, 1),
                #     ]

                pc_msg.fields = [
                    PointField('dist_x', 0, PointField.FLOAT32, 1),
                    PointField('dist_y', 4, PointField.FLOAT32, 1),
                    PointField('vrel_x', 8, PointField.FLOAT32, 1),
                    PointField('vrel_y', 12, PointField.FLOAT32, 1),
                    PointField('rcs', 16, PointField.FLOAT32, 1),
                    PointField('width', 20, PointField.FLOAT32, 1),
                    PointField('height', 24, PointField.FLOAT32, 1),
                    PointField('angle', 28, PointField.FLOAT32, 1),
                    PointField('prob', 32, PointField.INT32, 1),
                    PointField('id', 36, PointField.INT32, 1),
                    PointField('dyn_prop', 40, PointField.INT32, 1),
                    PointField('class', 44, PointField.INT32, 1),
                ]
                pc_msg.is_bigendian = False
                pc_msg.point_step = 48
                pc_msg.row_step = pc_msg.point_step * pc_msg.width
                pc_msg.is_dense = False

                # dtypes = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                #           ('dyn_prop', 'i1'), ('id', 'i2'), ('rcs', 'f4'),
                #           ('vx', 'f4'), ('vy', 'f4'), ('vx_comp', 'f4'), ('vy_comp', 'f4'),
                #           ('is_quality_valid', 'i1'), ('ambig_state', 'i1'),
                #           ('x_rms', 'i1'), ('y_rms', 'i1'), ('invalid_state', 'i1'),
                #           ('pdh0', 'i1'), ('vx_rms', 'i1'), ('vy_rms', 'i1')]

                dtypes = [('dist_x', 'f4'), ('dist_y', 'f4'), ('vrel_x', 'f4'), ('vrel_y', 'f4'),
                    ('rcs', 'f4'), ('width', 'f4'), ('height', 'f4'), ('angle', 'f4'),
                    ('prob', 'i4'), ('id', 'i4'), ('dyn_prop', 'i4'), ('class', 'i4')
                ]
                pc_arr = np.array(np.zeros(pc_msg.width), dtype=dtypes)

                for i, p in enumerate(msg.rps):
                    # pc_arr[i] = [p.distX, p.distY, 0,
                    #              0, p.id, p.rcs,
                    #              p.vx, p.vy, 0, 0,
                    #              0, 3,
                    #              0, 0, 0,
                    #              0, 0, 0]
                    pc_arr[i] = (p.distX, p.distY, p.vrelX, p.vrelY,
                                 p.rcs, p.width, p.height, p.angle,
                                 p.prob, p.id, p.dynProp, p.classT)
                
                pc_msg.data = pc_arr.tobytes()
                msg = pc_msg
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
                    ARGS.raw_camera,
                    True,
                )
    except KeyboardInterrupt:
        pass


# explicitly define the outward facing API of this module
__all__ = [decompress.__name__]

"""
python3 decompress.py -i <in.bag> -o <out.bag> -c /rgb/front_center/compressed_image /rgb/front_left/compressed_image /rgb/front_right/compressed_image /rgb/rear_center/compressed_image /rgb/rear_left/compressed_image /rgb/rear_right/compressed_image /rgb/right_mirror/compressed_image /rgb/left_mirror/compressed_image /rgb/thermal/compressed_image -r /rgb/front_center/original_image /rgb/front_left/original_image /rgb/front_right/original_image /rgb/rear_center/original_image /rgb/rear_left/original_image /rgb/rear_right/original_image /rgb/right_mirror/original_image /rgb/left_mirror/original_image /rgb/thermal/original_image -I ""
"""
