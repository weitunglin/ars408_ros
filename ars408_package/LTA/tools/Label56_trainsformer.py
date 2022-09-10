"""
Tusimple的資料標註格式的H_samples在label工具中提到是固定的56個y座標，其實還有
48個y座標的H_sample格式，從y=240~y=710。但是這個格式不能為我們的訓練所用，所以
必須要轉換成56的。

這個程式就是用來轉換的。
"""

import json
import argparse
h_sample56 = [160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710]
left_pts = 56-48


def arg():
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', type=str, help='json_path ')
    parser.add_argument('--edit', type=str, help='json_path ')
    args = parser.parse_args()
    return args
def main():
    print("start transforming...")
    args = arg()
    json_path = args.path
    edit_path = args.edit
    with open(json_path, 'r') as f:
        with open(edit_path, 'w') as e:
            while True:
                data = f.readline()
                if data =="":
                    break
                dict_f = json.loads(data)
                dict_f["h_samples"] = h_sample56
                lanes = dict_f["lanes"]
                lane56 = [-2 for i in range(left_pts)]
            
                index = 0
                for i in range(len(lanes)):
                    lanes[i] = [*lane56,*lanes[i]]
                dict_f["lanes"] = lanes
                print("dict = ",dict_f)
                # write into new json file
                print("writing dict = ",dict_f)
                json.dump(dict_f,e)    
                e.write('\n')
            e.close()
        f.close()


if __name__ == '__main__':
    main()



