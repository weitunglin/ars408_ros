import numpy as np
import cv2
import os
from tqdm import tqdm
"""
store data in the following form:
DataAugment.img_save_path : "/path/where/img/are/to/be/saved/"
DataAugment.dataset_save_path: "/path/where/dataset/is/to/be/saved/"
DataAugment.label_data_augmented = [
    .
    .
    label_data_augmented[i] = {
        img: img (numpy array)
        img_name : "imgname"
    Lanes: lane
    }
    .
    .
]

"""
class DataAugment:
    def __init__(self,labelloader,imglist = None):
        self.img_list = imglist
        self.labelloader = labelloader
        
        dataset_name = self.labelloader.data_path.split("/")[-1]
        dirpath = os.path.join("./data/augment",dataset_name)
        self.dir = dirpath
        self.dataset_save_path = dirpath
        print("dataset_save_path=",self.dataset_save_path)
        self.img_save_path  = dirpath+"/gt_image"
        print("img_save_path = ",self.img_save_path)
        self.label_data_augmented = [] #init
        
        #if not os.path.exists(dirpath):
        #    os.makedirs(dirpath)
    def mod(self,a,b):
        """
        a%b = return
        """
        a=a*10
        b=b*10
        return int(int(a)%int(b))
    def aug(self,shift_time = 1, shift_direction  = "all"):
        if shift_direction.lower() == "left" or "all":
            shifted_img = self.shift_left(shift_time)
        elif shift_direction.lower() == "right" or "all":
            shifted_img = self.shift_right(shift_time)
        else:
            raise ValueError("invalid direction")
        
    def shift_right(self,shift_time):
        
        if shift_time > 3: shift_time = 3
        elif shift_time < 0: shift_time = 1
        shift_list = [5,2,1.5]
        shifted_img = []
        
        print("load data from ",self.labelloader.data_path)
        #return
        bar = tqdm(total = len(self.labelloader.data_list))
        for data in self.labelloader.data_list:
            raw_file_path = self.labelloader.data_path+"/"+data["raw_file"]
            #print("orig",raw_file_path)

            img_ori = cv2.imread(raw_file_path)
            for t in range(shift_time):
                rows = []
                img = img_ori.copy()
                shift_th = shift_list[t]
                img_name_pre = "shiftright_{}_".format(t+1)
                raw_file_path.replace("\\","/")
                img_name = raw_file_path.split("/")[-1]
                img_name = img_name_pre + img_name
                augment_lanes = []
                for lane in data["lanes"]:
                    lane_copy = lane.copy()
                    augment_lanes.append(lane_copy)
                
                #print("augmenting {}...".format(img_name))
                #print("augment_lanes  = ",augment_lanes )
                
                label_data_augmented = {
                    "img": None,
                    "img_name" : img_name,
                    "Lanes" : None
                }
                shift_x = 1
                h_sample_index = 0
                for y in range(0, img.shape[0]):
                    if y >= 240 :
                        row_l = np.reshape(img[y,0:shift_x:1,:],(abs(shift_x),3))
                        #row_l = np.zeros(row_l.shape)
                        row_t = img[y,shift_x:,:]
                        row_a = np.reshape(np.average(row_t[:10],axis = 0),(1,3))
                        row_a = row_a.astype(np.uint8)
                        #print("avarge.shape = ",row_a.shape)
                        row_ll = np.zeros(row_l.shape,dtype = np.uint8)
                        label_shift = int(row_l.shape[0])
                        row_ll = row_ll + row_a
                        row_shift = np.append(row_t,row_ll,axis = 0)
                        if y%10==0: 
                            for lane in augment_lanes:
                                if lane[h_sample_index] >= 0:
                                    lane[h_sample_index] -= label_shift
                                if lane[h_sample_index]<0:lane[h_sample_index] = -2
                            #print("y= {},shift = {}".format(y,shift_x))
                            
                        if  self.mod(y,shift_th) == 0:
                            shift_x = shift_x +1
                            #print("shift_x = ", shift_x)
                    else: 
                        row_shift = img[y,:,:]
                    if y >=160 and y%10==0:
                        h_sample_index += 1
                    rows.append(row_shift)
                
                img_n = np.stack([row for row in rows])  
                label_data_augmented["img"] = img_n
                label_data_augmented["Lanes"] = augment_lanes
                print("test")
                self.test( label_data_augmented)
                
                self.label_data_augmented.append(label_data_augmented)
                shifted_img.append(img_n)
            bar.update(1)
        return shifted_img
    def shift_left(self,shift_time):
        
        if shift_time > 3: shift_time = 3
        elif shift_time < 0: shift_time = 1
        shift_list = [5,2,1.5]
        shifted_img = []
        
        print("load data from ",self.labelloader.data_path)
        #return
        bar = tqdm(total = len(self.labelloader.data_list))
        for data in self.labelloader.data_list:
            raw_file_path = self.labelloader.data_path+"/"+data["raw_file"]
            #print("orig",raw_file_path)

            img_ori = cv2.imread(raw_file_path)
            for t in range(shift_time):
                rows = []
                img = img_ori.copy()
                shift_th = shift_list[t]
                img_name_pre = "shiftleft_{}_".format(t+1)
                raw_file_path.replace("\\","/")
                img_name = raw_file_path.split("/")[-1]
                img_name = img_name_pre + img_name
                augment_lanes = []
                for lane in data["lanes"]:
                    lane_copy = lane.copy()
                    augment_lanes.append(lane_copy)
                
                #print("augmenting {}...".format(img_name))
                #print("augment_lanes  = ",augment_lanes )
                
                label_data_augmented = {
                    "img": None,
                    "img_name" : img_name,
                    "Lanes" : None
                }
                shift_x = -1
                h_sample_index = 0
                for y in range(0, img.shape[0]):
                    if y >= 240 :
                        row_l = np.reshape(img[y,-1:shift_x-1:-1,:],(abs(shift_x),3))
                        #row_l = np.zeros(row_l.shape)
                        row_t = img[y,0:shift_x,:]
                        row_a = np.reshape(np.average(row_t,axis = 0),(1,3))
                        row_a = row_a.astype(np.uint8)
                        #print("avarge.shape = ",row_a.shape)
                        row_ll = np.zeros(row_l.shape,dtype = np.uint8)
                        label_shift = int(row_l.shape[0])
                        row_ll = row_ll + row_a
                        row_shift = np.append(row_ll,row_t,axis = 0)
                        if y%10==0:
                            shifted_left_x = abs(shift_x) 
                            for lane in augment_lanes:
                                if lane[h_sample_index] >= 0:
                                    lane[h_sample_index] += label_shift
                                if lane[h_sample_index]>1279:lane[h_sample_index] = -2
                            #print("y= {},shift = {}".format(y,shift_x))
                            
                        if self.mod(y,shift_th) == 0:
                            shift_x = shift_x -1
                            #print("shift_x = ", shift_x)
                    else: 
                        row_shift = img[y,:,:]
                    if y >=160 and y%10==0:
                        h_sample_index += 1
                    rows.append(row_shift)
                
                img_n = np.stack([row for row in rows])  
                label_data_augmented["img"] = img_n
                label_data_augmented["Lanes"] = augment_lanes
                #self.test( label_data_augmented)
                
                self.label_data_augmented.append(label_data_augmented)
                shifted_img.append(img_n)
            bar.update(1)
        return shifted_img
    def test(self,label_data_augmented):
        lane_img = label_data_augmented["img"]
        
        h_sample = self.labelloader.data_list[0]["h_samples"]
        for lane in label_data_augmented["Lanes"]:
            i = 0
            x_pre = 0
            y_pre = 0
            i_th = h_sample.index(240)
            for x,y in zip(lane,h_sample):
                if i>i_th and i<55 and x>0 and y>0 and x_pre>0 and y_pre>0 :
                    cv2.line(lane_img,(x_pre,y_pre),(x,y),(0,0,255),3)
                x_pre = x
                y_pre = y    
                i+=1
        
        cv2.imshow("out",lane_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()       
        return lane_img
        
        
if __name__ == '__main__':
    path = './tools/data_augmentation/image'
    pth_list = os.listdir(path)
    image_list = []
    for i in pth_list:
        Full_path = os.path.join(path,i)
        image_list.append(cv2.imread(Full_path))
    Test = DataAugment(image_list)
    result = Test.shift_right(shift_time = 3)
   
    for img1,img2 in zip(image_list,result):
        img2 = img2.astype(np.uint8)
        cv2.imshow("out",img1)
        cv2.imshow("out2",img2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
        