import numpy as np
import torch.nn as nn
import torch
import time


def plot_bev(pointcloud, side_range, fwd_range, res, im, im_label):
    # 筛选出一定高度范围的点
    pointcloud = pointcloud[(pointcloud[:, 2] > -5) & (pointcloud[:, 2] < 0.5)]
    x_points = pointcloud[:, 0]
    y_points = pointcloud[:, 1]
    z_points = pointcloud[:, 2]
    # label = pointcloud[:, 3]
    
    # 获得区域内的点
    f_filt = np.logical_and(x_points > fwd_range[0], x_points < fwd_range[1])
    s_filt = np.logical_and(y_points > side_range[0], y_points < side_range[1])
    filter = np.logical_and(f_filt, s_filt)
    indices = np.argwhere(filter).flatten()
    x_points = x_points[indices]
    y_points = y_points[indices]
    z_points = z_points[indices]
    # label_points = label[indices]
    
    # 计算均值
    mean_std = np.array([[np.mean(x_points), np.var(x_points)],
                [np.mean(y_points), np.var(y_points)],
                [np.mean(z_points), np.var(z_points)]])
    print("mean and std:", mean_std)
    
    x_img = (-y_points / res).astype(np.int32)
    y_img = (-x_points / res).astype(np.int32)
    # 调整坐标原点
    x_img -= int(np.floor(side_range[0]) / res)
    y_img += int(np.floor(fwd_range[1]) / res)
    
    im[y_img, x_img] = z_points
    # im[y_img, x_img, 1] = y_points
    # im[y_img, x_img, 2] = z_points
    
    # im_label[y_img, x_img] = label_points
    
    return im


def main(pointcloud):
    # 设置鸟瞰图范围
    side_range = (-30, 30)  # 左右距离
    fwd_range = (-30, 30)  # 后前距离
    res = 0.1  # 分辨率
    # 创建图像数组
    x_max = 1 + int((side_range[1] - side_range[0]) / res)
    y_max = 1 + int((fwd_range[1] - fwd_range[0]) / res)
    im = np.zeros([y_max, x_max], dtype=np.uint8)
    
    im = plot_bev(pointcloud, side_range, fwd_range, res, im)
    
    tensor = torch.from_numpy(im).unsqueeze(0).unsqueeze(0).to(torch.float32)
    kernel_size = 3
    pad_size = int((kernel_size - 1) / 2)
    dilate_op = nn.MaxPool2d(kernel_size, 1, pad_size)
    dilate_bev = dilate_op(tensor)
    dilate_bev = dilate_bev.squeeze().numpy()
    
    # cmap = plt.cm.colors.ListedColormap(['black', 'green', 'yellow', 'white'])
    #
    # _, axs = plt.subplots(1, 2, figsize=(9, 9))
    #
    # axs[0].imshow(dilate_bev, cmap=cmap, interpolation='nearest')
    # axs[1].imshow(im, cmap=cmap, interpolation='nearest')
    # plt.show()


if __name__ == "__main__":
    # 点云读取
    file_path = '/home/robotics/carla_selfdriving_ws/carla_client/data/lidar_project.txt'
    # 使用numpy.loadtxt读取文件并生成二维数组
    pointcloud = np.loadtxt(file_path)
    
    while True:
        t0 = time.time()
        main(pointcloud)
        print(time.time() - t0)
