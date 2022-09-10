
from data_augmentation.Augment import DataAugment
from data_augmentation.LabelLoader import  LabelLoader
from data_augmentation.Saver import Augmented_Saver
import os
import argparse
def args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--root', 
        type=str, 
        default="./data/6_8_11",
        help='path of training data dir'
    )
    #parser.add_argument('--source', type=str, default='inference/videos', help='source')  # file/folder   ex:inference/images
    args = parser.parse_args()
    return args
def main(args):
    path = args.root
    labelloader = LabelLoader(path)
    labelloader.data_orgnize()
    
    DataAugmentation = DataAugment(labelloader)
    DataAugmentation.shift_right(shift_time=3)
    Saver = Augmented_Saver(DataAugmentation)
    Saver.Save_Augment()
if __name__ == '__main__':
    args = args()
    main(args)