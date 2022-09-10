import json
import os
import cv2

class Augmented_Saver:
    def __init__(self,DataAugmentation):
        ROOT = "./data/augment"
        self.label_data_augmented = DataAugmentation.label_data_augmented
        self.save_dir = DataAugmentation.dataset_save_path
        self.img_save_dir = DataAugmentation.img_save_path
        self.h_sample = DataAugmentation.labelloader.data_list[0]["h_samples"]
        self.mkdir()
    def mkdir(self):
        if os.path.exists(self.save_dir):
            self.save_dir = self.save_dir+"_2"
        os.mkdir(self.save_dir)
        if os.path.exists(self.save_dir):
            os.mkdir(self.img_save_dir)
        #make jsonfile
        file = self.save_dir + "/" + "JsonFile_label.json"
        self.JsonFile_label = file
        open(file, 'a').close()
    def Save_Augment(self):
        print("Save Augmented Data")
        for Augment_Data in self.label_data_augmented:
            JsonElement = {
                "lanes": [],
                "h_samples":self.h_sample, 
                "raw_file": ""
                }
            #save_img
            img = Augment_Data["img"]
            img_path = os.path.join(self.img_save_dir,Augment_Data["img_name"])
            cv2.imwrite(img_path,img)
            JsonElement["lanes"] = Augment_Data["Lanes"]
            JsonElement["raw_file"] = "gt_image/"+Augment_Data["img_name"]
            self.JsonOutput(JsonElement)
        file = self.save_dir + "/" + "valset.json"
        open(file, 'a').close()
        file = self.save_dir + "/" + "test_label.json"
        open(file, 'a').close()
    def JsonOutput(self,JsonElement):
        with open(self.JsonFile_label, 'a') as f:
            json.dump(JsonElement, f)
            f.write('\n')      

if __name__ == '__main__':
    print("I am the bone of my sword")

