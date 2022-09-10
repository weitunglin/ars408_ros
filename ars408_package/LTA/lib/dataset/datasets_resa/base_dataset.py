import os.path as osp
import os
import numpy as np
import cv2
import torch
from torch.utils.data import Dataset
import torchvision
import lib.utils_resa.transforms as tf
from .registry import DATASETS

H_sample = [160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710]
@DATASETS.register_module
class BaseDataset(Dataset):
    def __init__(self, img_path, data_list, list_path='list', cfg=None):
        self.cfg = cfg
        self.img_path = img_path
        self.list_path = osp.join(img_path, list_path)
        self.data_list = data_list
        self.is_training = ('train' in data_list)

        self.img_name_list = []
        self.full_img_path_list = []
        self.label_list = []
        self.exist_list = []

        self.transform = self.transform_train() if self.is_training else self.transform_val()

        self.init()

    def transform_train(self):
        raise NotImplementedError()

    def transform_val(self):
        val_transform = torchvision.transforms.Compose([
            tf.SampleResize((self.cfg.img_width, self.cfg.img_height)),
            tf.GroupNormalize(mean=(self.cfg.img_norm['mean'], (0, )), std=(
                self.cfg.img_norm['std'], (1, ))),
        ])
        return val_transform
    def coords_denoise(self,coords):
        slope_th = 3 # dx/dy should less than 200
        coord_index = 0
        for coord in coords:
            candidate = [[],[]] 
            pointer = 0
            state = False
            x1 = 0
            y1 = 0
            for x,y in coord:
                pre_slope_state = "0"
                slope_state = "" # + - 0
                if x>0 and  y>0:
                    if not state:
                        x1 = x
                        y1 = y
                        state = True
                    if state:
                        dx = x-x1
                        dy = y-y1
                        x1 = x
                        y1 = y
                        if abs(dx/dy) > slope_th and dx>0:
                            slope_state = "+"
                        elif abs(dx/dy) >slope_th and dx<0:
                            slope_state = "-"
                        else:   
                            slope_state = "0"
                        if slope_state!= "0" and slope_state!= pre_slope_state:
                            pointer = (pointer+1)%2 # 0,1
                        elif slope_state =="0":
                            candidate[pointer].append([x,y]) 
                        pre_slope_state = slope_state
                        candidate[pointer].append([x,y])    
            if len(candidate[0]) > len(candidate[1]):
                coords[coord_index] = candidate [0]
            else:
                coords[coord_index] = candidate [1]
            coord_index+=1
        return coords
    def view(self, img, coords, file_path=None):
        #1 verify where we are driving the car, find the two lanes 
        #coords = self.coords_denoise(coords)
        center_x = 1280/2
        leftlane_starndar = 1280/2
        coord_index = 0
        #record = [[] for i in range(len(coords))]
        left = [] # [lane_index(int),lane(list)]
        right = [] # [lane_index(int),lane(list)]
        coords = sort_key(coords)
        for coord in coords:
            total_pt_number = 0
            x_Sum = 0
            for i in range(56):
                if coord[i][0]>0 and coord[i][1]>=300:
                    total_pt_number+=1
                    x_Sum+= coord[i][0]
            if x_Sum/total_pt_number <= leftlane_starndar:
                coord_index+=1
            else:
                break
        if len(coords)==0: # no lane
            cv2.putText(
                        img, "no line detected", (640,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),1, cv2.LINE_AA
                        )
            left.append(-1)
            right.append(-1)
        elif coord_index == 0: # no left lane
            cv2.putText(
                        img, "left", (640,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),1, cv2.LINE_AA
                        )
            left.append(-1)
            right.append(0)
            right.append(coords[0])
        elif coord_index == len(coords) and len(coords) != 0:# no right lane
            cv2.putText(
                        img, "right", (640,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),1, cv2.LINE_AA
                        ) 
            right.append(-1)
            left.append(len(coords)-1)
            left.append(coords[-1])
        else : # both left and right have lane
            cv2.putText(
                        img, "center", (640,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),1, cv2.LINE_AA
                        ) 
            right.append(coord_index)
            right.append(coords[coord_index])
            left.append(coord_index-1)
            left.append(coords[coord_index-1])
        #y_sam = [x for x in range(360,720,10)].reverse()
        color = [
            (255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(255,255,0)
        ]
        color_index = 0
        lane_index = 0
        
        for coord in coords:
            if lane_index == left[0] or lane_index ==right[0]:
                drawline = False
                for x, y in coord:
                    if x <= 0 or y <= 0:
                        continue
                    x, y = int(x), int(y)
                    if not drawline:
                        x1 = x
                        y1 = y
                        drawline = True
                    else:
                        x2 = x
                        y2 = y
                        cv2.line(img,(x1,y1),(x2,y2),color[color_index],5)
                        x1 = x2
                        y1 = y2
                    i+=1
            else:
                for x, y in coord:
                    if x <= 0 or y <= 0:
                        continue
                    x, y = int(x), int(y)
                    cv2.circle(img, (x, y), 4, color[color_index], 2)    
            color_index += 1
            lane_index += 1
        if file_path is not None:
            if not os.path.exists(osp.dirname(file_path)):
                os.makedirs(osp.dirname(file_path))
            cv2.imwrite(file_path, img)




    def init(self):
        raise NotImplementedError()


    def __len__(self):
        return len(self.full_img_path_list)

    def __getitem__(self, idx):
        #print("full_img_path_list[idx] = ",self.full_img_path_list[idx])
        #raise NotImplementedError
        img = cv2.imread(self.full_img_path_list[idx]).astype(np.float32)
        img = img[self.cfg.cut_height:, :, :]

        if self.is_training:
           # print("self.label_list[idx] = ",self.label_list[idx])
            path_temp = self.label_list[idx].split('/')
            if path_temp[-2][-4:] == ".png" or path_temp[-2][-4:] == ".jpg": #remove ".png"
                path_temp[-2] = path_temp[-2][:-4]
                path = "."
                for str in path_temp[1:]:
                    path += "/"
                    path = path + str
            else:
                path = self.label_list[idx]      

            #print("path = ",path)
            label = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        
            if len(label.shape) > 2:
                label = label[:, :, 0]
            label = label.squeeze()
            label = label[self.cfg.cut_height:, :]
            exist = self.exist_list[idx]
            if self.transform:
                img, label = self.transform((img, label))
            label = torch.from_numpy(label).contiguous().long()
        else:
            img, = self.transform((img,))

        img = torch.from_numpy(img).permute(2, 0, 1).contiguous().float()
        meta = {'full_img_path': self.full_img_path_list[idx],
                'img_name': self.img_name_list[idx]}

        data = {'img': img, 'meta': meta}
        if self.is_training:
            data.update({'label': label, 'exist': exist})
        return data

def sort_key(coords):
    
    for i in range(56):
        a = 0
        for coord in coords:
            if  coord[i][0] >0 :# 有點
                a+=1
        if a == len(coords):
            break
      
    
    coords.sort(key = lambda x: x[i][0] ) 

    return coords
