import os
import json
import glob
"""
Object:
------------
LabelLoader:

attibute:
------------------
Labelloader.data_path
Labelloader.data_list
Labelloader.json_list

"""
class LabelLoader:
    def __init__(self,path):
        self.data_path =path
        
        self.json_loader(path)
    def json_loader(self,path):
        #os.chdir(self.data_path)
        self.data_path
        self.json_list = glob.glob(self.data_path+"/*.json")
        if len(self.json_list)==0:
            raise Exception("No json file exist")
    def data_orgnize(self,):
        """
        store every gt_data in the following form：
        data = {"raw_flie":gt_image_path, "label":lanes}
        """
        data_list = []
        print('self.json_list=',self.json_list)
        for json_path in self.json_list:
            if "valset.json" in json_path or "test_label.json" in json_path:
                continue
            with open(json_path, 'r')  as f:
                print("reading data in json_path = ",json_path) 
                while True:
                    data = f.readline()
                    if data=="":
                        break
                    #print("read ",data)
                    dict_f = json.loads(data)
                    data_list.append(dict_f)
                f.close()
        self.data_list = data_list