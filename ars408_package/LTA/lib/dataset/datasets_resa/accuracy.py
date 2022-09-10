

#copy from 1.https://github.com/ZJULearning/resa.git
def accruacy_func(x_pred,x_gt):
    x_gt = float(x_gt)
    x_diff = abs(float(x_pred-x_gt))
    accuracy = (x_diff/x_gt)


    return accuracy
def accuracy_cal(pred_lane_coords,gt_lane_coords):
    lane_accr = []

    for coord_pred , coord_gt in zip(pred_lane_coords, gt_lane_coords):
        for x_pred,x_gt in zip(coord_pred, coord_gt):
            pt_accuracy = pt_accuracy + accruacy_func(x_pred, x_gt)
        mean_accr = pt_accuracy/len(coord_pred)
        lane_accr.append(mean_accr)
    return lane_accr