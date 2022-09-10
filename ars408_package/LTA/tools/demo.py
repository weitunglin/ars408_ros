import argparse
import os, sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)
os.chdir(BASE_DIR)
print(sys.path)
import torch
from lib.config import cfg
from lib.models import get_net
from lib.utils_resa.config import Config
from lib.runner.demo_runner import DemoString


def detect(cfg,opt):
    #載入model----------------------------------------------------
    model = get_net(cfg)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    #載入weight---------------------------------------------------
    print("loading checkpoint file from ",opt.weights)
    checkpoint = torch.load(opt.weights, map_location= device)
    model.load_state_dict(checkpoint['net'],strict = False)
    model = model.to(device)
    # Load Config file--------------------------------
    cfg_resa = Config.fromfile('./lib/config/resa_tusimple.py')
    cfg_resa.gpus = opt.gpus
    cfg_resa.load_from = opt.load_from
    cfg_resa.finetune_from = opt.finetune_from
    cfg_resa.view = opt.view
    cfg_resa.work_dirs = opt.work_dirs + '/' + 'TuSimple'
    opt.load_from = opt.weights
    #------------------------------------

    dir = opt.demo_data_path #demo資料的資料夾
    data_list = os.listdir(dir)
    if opt.source_type.lower() == "video":# demo_data是影片的情況 
        for path in data_list:
            p = opt.demo_data_path +"/{}".format(path)
            print("loading {} ...".format(path))
            #initial your demo data    
            DemoRunner = DemoString(p,model,checkpoint,device,cfg_resa,opt.source_type)
            #run demo 
            DemoRunner.run()
    elif opt.source_type.lower() == "image":#  demo_data是圖片的情況
        p = dir
        #initial your demo data 
        DemoRunner = DemoString(p,model,checkpoint,device,cfg_resa,opt.source_type)
        #run demo
        DemoRunner.run()
    else:
        raise Exception("invalid source_type")
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str,default= cfg.MODEL.PRETRAINED, help='model.pth path(s)')
    parser.add_argument('--source', type=str, default='inference/videos', help='source')  # file/folder   ex:inference/images
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='gpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--load_from' ,type = str)
    parser.add_argument('--finetune_from', default=None,help='whether to finetune from the checkpoint')
    parser.add_argument('--view',action='store_true',help='whether to show visualization result')
    parser.add_argument('--gpus', nargs='+', type=int, default='1')
    parser.add_argument('--source_type', type = str , default='image', help = "image or video")
    parser.add_argument('--demo_data_path', type = str,default = "./demo", help = "a path of a dir to your demo data ")
    parser.add_argument('--work_dirs', type=str, default='work_dirs',help='work dirs')
    opt = parser.parse_args()
    with torch.no_grad():
        detect(cfg,opt)
