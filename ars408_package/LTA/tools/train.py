"""
MODIFICATION NOTICES
THIS NOTICE IS BASED ON LICENSE_RESA
THIS FILE HAS BEEN MODIFIED  
ORIGIN CODE: https://github.com/ZJULearning/resa.git
"""

import argparse
import os, sys
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)
import torch
import torch.nn.parallel
from torch.cuda import amp
import torch.distributed as dist
import torch.backends.cudnn as cudnn
import torch.optim
import torch.utils.data
import torch.utils.data.distributed
from lib.config import cfg
from lib.config import update_config
from lib.models import get_net
# resanet
from lib.runner.runner import Runner
from lib.dataset.datasets_resa import build_dataloader
from lib.utils_resa.config import Config
def parse_args():
    parser = argparse.ArgumentParser(description='Train Multitask network')
    parser.add_argument('--modelDir',
                            help='model directory',
                            type=str,
                            default='')
    parser.add_argument('--logDir',
                            help='log directory',
                            type=str,
                            default='runs/')
    parser.add_argument('--dataDir',
                            help='data directory',
                            type=str,
                            default='')
    parser.add_argument('--prevModelDir',
                            help='prev Model directory',
                            type=str,
                            default='')
    parser.add_argument('--sync-bn', action='store_true', help='use SyncBatchNorm, only available in DDP mode')
    parser.add_argument('--local_rank', type=int, default=-1, help='DDP parameter, do not modify')
    parser.add_argument('--conf-thres', type=float, default=0.001, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.6, help='IOU threshold for NMS')    
    
   
    #parser = argparse.ArgumentParser(description='Train a detector')
    #parser.add_argument('config', type = str ,default='lib/configs/tusimple.py')
    parser.add_argument(
        '--work_dirs', type=str, default='work_dirs',
        help='work dirs')
    parser.add_argument(
        '--load_from', default=cfg.MODEL.PRETRAINED,
        help='the checkpoint file to resume from')
    parser.add_argument(
        '--finetune_from', default=None,
        help='whether to finetune from the checkpoint')
    parser.add_argument(
        '--validate',
        action='store_true',
        help='whether to evaluate the checkpoint during training')
    parser.add_argument(
        '--view',
        action='store_true',
        help='whether to show visualization result')
    parser.add_argument('--gpus', nargs='+', type=int, default='0')
    parser.add_argument('--seed', type=int,
                        default=None, help='random seed')
    parser.add_argument('--demo',action='store_true',
        help='whether to demo the checkpoint during training')
    args = parser.parse_args()
    return args

def main():
    # set all the configurations
    from lib.config import cfg
    args = parse_args()
    update_config(cfg, args)
    
    
    # bulid up model
    print("begin to bulid up model...")
    print("load model to device")
    device = torch.device("cuda:{}".format(args.gpus[0]) if torch.cuda.is_available() else "cpu")
    model = get_net(cfg).to(device)
    """
    model 的東西寫在lib/models/YOLOP.py 裡面
    """
    #load pretrained model
    if os.path.exists(cfg.MODEL.PRETRAINED):
        print("=> loading model '{}'".format(cfg.MODEL.PRETRAINED))
        checkpoint = torch.load(cfg.MODEL.PRETRAINED)
        model.load_state_dict(checkpoint,strict = False)
            
    print("begin to load data")
    # Data loading
    args = parse_args()
    os.environ["CUDA_VISIBLE_DEVICES"] = ','.join(str(gpu) for gpu in args.gpus)
    #print('args.config=',args.config)
    cfg_resa = Config.fromfile('./lib/config/resa_tusimple.py')
    #cfg = Config.fromfile('./configs/tusimple.py')
    cfg_resa.gpus = len(args.gpus)

    cfg_resa.load_from = args.load_from
    cfg_resa.finetune_from = args.finetune_from
    cfg_resa.view = args.view
    
    cfg_resa.work_dirs = args.work_dirs + '/' + 'TuSimple'

    cudnn.benchmark = True
    cudnn.fastest = True
    
    runner = Runner(cfg_resa,model)

    if args.validate:
        val_loader = build_dataloader(cfg_resa.dataset.val, cfg_resa, is_train=False)
        print("validate....")
        runner.validate(val_loader)
    else:
        runner.train()
        
  


if __name__ == '__main__':
    main()