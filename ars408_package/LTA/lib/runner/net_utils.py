"""
MODIFICATION NOTICES
THIS NOTICE IS BASED ON LICENSE_RESA
THIS FILE HAS BEEN MODIFIED 
ORIGIN CODE: https://github.com/ZJULearning/resa.git
"""

from glob import escape
import torch
import os
from torch import nn
import numpy as np
import torch.nn.functional
from termcolor import colored
from .logger import get_logger
from lib.config import cfg

def save_model(net, optim, scheduler, recorder, dataset_path, is_best=False):
    model_dir = os.path.join(recorder.work_dir, 'ckpt')
    #os.system('mkdir -p {}'.format(model_dir))
    if not os.path.isfile(model_dir):
        try:
            os.makedirs(model_dir)
        except:
            pass
    epoch = recorder.epoch
    ckpt_name = 'best' if is_best else epoch
    torch.save({
        'net': net.state_dict(),
        'optim': optim.state_dict(),
        'scheduler': scheduler.state_dict(),
        'recorder': recorder.state_dict(),
        'epoch': epoch
    }, os.path.join(model_dir, '{}.pth'.format(ckpt_name)))
    if ckpt_name == "best":
        save_dir_path = "./weights"
        dataset_path.replace("\\","/")
        ckpt_name = dataset_path.split("/")[-1]
        torch.save({
            'net': net.state_dict(),
            'optim': optim.state_dict(),
            'scheduler': scheduler.state_dict(),
            'recorder': recorder.state_dict(),
            'epoch': epoch
        }, os.path.join(save_dir_path, '{}.pth'.format(ckpt_name)))

def load_network_specified(net, model_dir, logger=None):
    pretrained_net = torch.load(model_dir)['net']
    net_state = net.state_dict()
    state = {}
    for k, v in pretrained_net.items():
        if k not in net_state.keys() or v.size() != net_state[k].size():
            if logger:
                logger.info('skip weights: ' + k)
            continue
        state[k] = v
    net.load_state_dict(state, strict=False)


def load_network(net, model_dir, finetune_from=None, logger=None):
    if finetune_from:
        if logger:
            logger.info('Finetune model from: ' + finetune_from)
        load_network_specified(net, finetune_from, logger)
        return
    pretrained_model = torch.load(model_dir)
    try:
        net.load_state_dict(pretrained_model['net'], strict=False)
    except :
        net.load_state_dict(pretrained_model,strict = False)