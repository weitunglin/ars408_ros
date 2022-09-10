import torch
import math
#copy from 1.https://github.com/ZJULearning/resa.git

_scheduler_factory = {
    'LambdaLR': torch.optim.lr_scheduler.LambdaLR,
}


def build_scheduler(cfg, optimizer):

    assert cfg.scheduler.type in _scheduler_factory

    cfg_cp = cfg.scheduler.copy()
    cfg_cp.pop('type')

    scheduler = _scheduler_factory[cfg.scheduler.type](optimizer, **cfg_cp)


    return scheduler 
