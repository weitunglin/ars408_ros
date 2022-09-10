"""
--------------------------------------
Lane Self-Correction System
--------------------------------------
Editor : Salmon
Editor2 : Stalin
"""
class LaneCorrector:
    def __init__(self,coords):
        self.coords = coords
    def Salmon_Fliter(self): #you can add argument if you want
        #Salmon's 
        
        coords = self.coords    #各車道線之xy座標組
        counts = 0  #車道線數量
        
        #車道線逐線處理；coord為單條車道線之xy座標組，共有56組。
        for coord in coords:
            counts += 1
            #print("第%d條線開始: " % counts, coord)
            
            #根據Filter所需參數進行初步的設定
            x0 = 1280/2     #預設狀態起始點
            p0 = 0          #狀態不確定度
            u0 = 0          #初始斜率
            set_flag = 0    #狀態0：初始設置未開始；狀態1：起始點已設置、斜率尚未確定；狀態2：初始設置完成。
            
            #取出各組xy座標，y軸為固定相差10，無需處理。
            for i in range(56):
                x = coord[i][0]
                #如果x座標小於0代表該點無線。
                if x <= 0:
                    continue
                #第一次取得有效xy座標組為車道線由上往下看的頭，進行初始設置。
                if set_flag == 0:
                    x0 = x
                    set_flag = 1
                elif set_flag == 1:
                    u0 = x-x0
                    x0 = x
                    set_flag = 2
                    #print("初始設置完成")
                
                #丟進濾波器處理並傳回coords
                else:
                    x = int(x)
                    z = x
                    x10, p10 = self.filter_predict(x0, p0, u0)
                    x1, p1, k = self.filter_update(x10, p10, z)
                    u0 = x1 - x0    #為下一組座標提供上一次的斜率。
                    x0 = x1         #為下一組座標提供上一次的x座標。
                    self.coords[counts-1][i][0] = x1  #回傳數據
        return self.coords
    # if you need, you can add more Function 
    def filter_predict(self, x0, p0, u0):
        '''
        Function:
            基於Kalman Filter架構。
            使用前一個狀態與其相關參數來預測現在的狀態。
        Input:
            x0為前一個點。
            p0為目前的不確定度。
            u0為狀態轉移關係，這裡為斜率。
        Output:
            x10為運用前一個時刻（點）預測出的下一個點。
            p10為這個東西有多不確定。
        '''
        q = 0.1 #狀態轉移變異數

        x10 = x0 + u0   #根據前一個狀態預測現在的座標
        p10 = p0 + q    #這個座標的不確定程度
        
        return (x10, p10)
    def filter_update(self, x10, p10, z):
        '''
        Function:
            基於Kalman Filter架構。
            利用現在觀測到的值，更新預測的狀態。
        Input:
            x10為先前預測值。
            p10為不確定度。
            z為觀測到的值。
        Output:
            x1是經過預測及更新，由Filter推測出原本系統應在的狀態。
            p1為不確定度。
            k為Kalman Gain，是用來代表這個觀測值的可信度有多少的參數，回報來偵錯。
        '''
        r = 0.1 #觀測誤差
        
        #極端觀測值不予採用
        if abs(z - x10) > 50:   #觀測值與預測值相差甚鉅，推定觀測值有誤。
            z = x10
        
        k = p10 / (p10 + r)         #Kalman gain
        x1 = x10 + k * (z - x10)    #根據觀測值更新x座標
        p1 = (1 - k) * p10          #現在狀態的不確定程度
        
        return (x1, p1, k)