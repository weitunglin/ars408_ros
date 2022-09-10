"""
MODIFICATION NOTICES
THIS NOTICE IS BASED ON LICENSE_RESA
THIS FILE HAS BEEN MODIFIED 
ORIGIN CODE: https://github.com/ZJULearning/resa.git
"""

import time
import torch
import numpy as np
from tqdm import tqdm
import pytorch_warmup as warmup
import cv2

from lib.RESANet.registry import build_net
from .registry import build_trainer, build_evaluator
from .optimizer import build_optimizer
from .scheduler import build_scheduler
from lib.dataset.datasets_resa import build_dataloader
from .recorder import build_recorder
from .net_utils import save_model, load_network
#user cdoe
import torchvision
import lib.utils_resa.transforms as tf

class Runner(object):
    def __init__(self, cfg , Net):
        self.cfg = cfg
        self.recorder = build_recorder(self.cfg)
        self.net = Net #build_net(self.cfg)
        #print("Net = ",Net)
        self.net = torch.nn.parallel.DataParallel(
                self.net, device_ids = range(self.cfg.gpus)).cuda()
        self.recorder.logger.info('Network: \n' + str(self.net))
        self.resume()
        self.optimizer = build_optimizer(self.cfg, self.net)
        self.scheduler = build_scheduler(self.cfg, self.optimizer)
        self.evaluator = build_evaluator(self.cfg)
        self.warmup_scheduler = warmup.LinearWarmup(
            self.optimizer, warmup_period=5000)
        self.metric = 0.

    def resume(self):
        if not self.cfg.load_from and not self.cfg.finetune_from:
            return
        load_network(self.net, self.cfg.load_from,
                finetune_from=self.cfg.finetune_from, logger=self.recorder.logger)

    def to_cuda(self, batch):
        for k in batch:
            if k == 'meta':
                continue
            batch[k] = batch[k].cuda()
        return batch
    
    def train_epoch(self, epoch, train_loader):
        
        self.net.train()
        end = time.time()
        max_iter = len(train_loader)
        #progress = tqdm(total=max_iter)
        for i, data in enumerate(train_loader):
            
            if self.recorder.step >= self.cfg.total_iter:
                break
            date_time = time.time() - end
            self.recorder.step += 1
            data = self.to_cuda(data)
            output = self.trainer.forward(self.net, data) #put training_data into Net
            self.optimizer.zero_grad()
            loss = output['loss']
            loss.backward()
            self.optimizer.step()
            self.scheduler.step()
            self.warmup_scheduler.dampen()
            batch_time = time.time() - end
            end = time.time()
            self.recorder.update_loss_stats(output['loss_stats'])
            self.recorder.batch_time.update(batch_time)
            self.recorder.data_time.update(date_time)

            if i % self.cfg.log_interval == 0 or i == max_iter - 1:
                lr = self.optimizer.param_groups[0]['lr']
                self.recorder.lr = lr
                self.recorder.record('train')
        #progress.update(1)
    def train(self):
        self.recorder.logger.info('start training...')
        self.trainer = build_trainer(self.cfg)
        print("load data from {}...".format(self.cfg.dataset.train['img_path']))
        train_loader = build_dataloader(self.cfg.dataset.train, self.cfg, is_train=True)
        val_loader = build_dataloader(self.cfg.dataset.val, self.cfg, is_train=False)

        for epoch in range(self.cfg.epochs):
            self.recorder.epoch = epoch
            self.train_epoch(epoch, train_loader)
            #self.save_ckpt()
            if (epoch + 1) % self.cfg.save_ep == 0 or epoch == self.cfg.epochs - 1:
                self.save_ckpt()
            if (epoch + 1) % self.cfg.eval_ep == 0 or epoch == self.cfg.epochs - 1:
                self.validate(val_loader)
            if self.recorder.step >= self.cfg.total_iter:
                break
        
    def validate(self, val_loader):
        self.net.eval()
        #print('val_loader=',len(val_loader))
        for i, data in enumerate(tqdm(val_loader, desc=f'Validate')):
            #print("data =",data["img"])
            data = self.to_cuda(data)
            with torch.no_grad():
                output = self.net(data['img'])[0]
                #print("output shape = ",output['seg'].shape)
                
                img = output['seg'].cpu().numpy()
                #print('img.shape =',img.shape)
                #cv2.imshow('img',img)
                #cv2.waitkey(0)
                try:
                    self.evaluator.evaluate(val_loader.dataset, output, data)
                except :
                    print("validate end") 
        metric = self.evaluator.summarize()
        if not metric:
            return
        if metric > self.metric:
            self.metric = metric
            self.save_ckpt(is_best=True)
        self.recorder.logger.info('Best metric: ' + str(self.metric))

    def save_ckpt(self, is_best=False):
        save_model(self.net, self.optimizer, self.scheduler,
                self.recorder, self.cfg.dataset_path, is_best)
    #demo.py會跑到這裡 -----------------
    def demo(self, img_path, img_list):
        import os
        self.net.eval()
        #print('val_loader=',len(val_loader))
        k = tqdm(len(img_list))
        for img_name in img_list:
            path = os.path.join(img_path, img_name)
            #print("path = ",path)
            image = cv2.imread(path).astype(np.float32)
            image = image[160:, :, :]
            val_transform = torchvision.transforms.Compose([
            tf.SampleResize((640, 384)),
            tf.GroupNormalize(mean=([103.939, 116.779, 123.68], (0, )), std=(
                [1., 1., 1.], (1, ))),
            ])
            image, = val_transform((image,))
            image = torch.from_numpy(image).permute(2, 0, 1).contiguous().float()
            image = image.unsqueeze(0)
            
            data = {
                'img':image,
                'meta':path
            }
            data = self.to_cuda(data)
            
            with torch.no_grad():
                output = self.net(data['img'])[0]
                img = output['seg'].cpu().numpy()
                try:
                    self.evaluator.demo(path, output, data)
                except :
                    pass
                    #print("demo end") 
            k.update(1)