from requests.sessions import TooManyRedirects
from cv2 import cv2
import time
import Pyro4
import numpy as np
# import tkinter as tk
from matplotlib import pyplot as plt

try:
    from grasp_2d import Grasp2D
    from tf_mapper import TFMapper_EyeInHand
    from camera import CameraBase
except Exception as e:
    print('import utils error')
    raise e

# 172.17.0.1
Pyro4.config.SERIALIZER = 'pickle'
grasp_server = Pyro4.Proxy("PYRO:grasp@192.168.1.202:6665")
Pyro4.asyncproxy(grasp_server)


def Plan(image):
    image = image.astype('float32')

    return grasp_server.plan(image, 70)


class GrapsPanler(object):
    # CROP_START = np.array([400, 200])  # 200, 50
    # CROP_SIZE = 250  # 480  480*270
    # CROP_START = np.array([140, 35])  # 640*480
    # CROP_SIZE = 200  # 480
    CROP_START = np.array([80, 0])  # 640*480
    CROP_SIZE = 480  # 480
    SCALE = 300 / CROP_SIZE

    def __init__(self, rgb, raw_depth):
        self.rgb = rgb
        self.crop_depth = self.process(raw_depth)
        # decimation = rs.spatial_filter()
        # self.image = decimation.process(self.image)
        self.min_depth, self.img_medianBlur = self.get_min_depth()
        self.depth = cv2.GaussianBlur(self.crop_depth, (3, 3), 0)
        # self.table_depth = cv2.GaussianBlur(self.process(table_depth),(3,3),0)
        self.mean = np.mean(self.depth)
        self.std = np.std(self.depth)
        print("pingjunzhi", self.mean)
        # cor:1.1981683 ; 0.32395524  #mean; std
        # gmd: 0.7338126;0.00363 ;  #30000: 0.69948566;0.005129744
        # jaq: 1.5008891 ; 0.04099764；#30000: 1.4984636 0.04409859
        # self.normalize_depth = (self.depth-self.mean*np.ones(self.depth.shape))/self.std  * 0.32395524 +1.1981683*np.ones(self.depth.shape)
        self.normalize_depth = (self.depth-self.mean*np.ones(self.depth.shape)) / \
            self.std * 0.005129744+0.69948566*np.ones(self.depth.shape)
        # self.normalize_depth = (self.depth-self.mean*np.ones(self.depth.shape))/self.std  * 0.04409859 +1.4984636*np.ones(self.depth.shape)
        # print("maxmaxmax:",np.max(self.depth))
        # self.grasp_pyro = Plan(self.depth-self.table_depth)#ggrot此处需要减去桌面
        self.grasp_pyro = Plan(self.depth)

    def get_min_depth(self):
        img_GaussianBlur = cv2.GaussianBlur(self.crop_depth, (5, 5), 0)
        img_medianBlur = cv2.medianBlur(img_GaussianBlur, 5)  # ,(5,5))
        a = img_medianBlur[np.nonzero(img_medianBlur)]
        b = np.array([n for n in a if n > 0.1])
        min_depth = np.min(b)
        # print('最小深度:', min_depth)
        return min_depth, img_medianBlur

    def is_ready(self):
        print("sh")
        return self.grasp_pyro.ready

    def get_grasp(self):
        # if not self.is_ready():
        #     return None
        print('---------------------------')
        print('服务器返回')
        print('---------------------------')
        try:
            result = self.grasp_pyro.value
        except Exception as e:
            for _ in range(20):
                print('\033[31;41m 规划失败!!!!!! \033[0m', e)
            return None
        print('服务器返回的结果为：', result)
        if result is None:
            return None

        p0, p1, d0, d1, q = result
        # plt.clf()
        # plt.axis('off')
        # plt.imshow(pic)
        # plt.show()
        self.result_p0 = p0
        self.result_p1 = p1
        self.result_center = (p0+p1)/2
        # 这里是要把在300x300的小图下的抓取点坐标还原到整个摄像头的原始视野中
        p0_temp = self.process_to_original(p0)
        p1_temp = self.process_to_original(p1)
        self.grasp = Grasp2D.from_endpoint(p0_temp, p1_temp, d0, d1, q)
        return self.grasp

    def calculate_grasp_depth(self):
        line_point = self.myline(
            self.result_p0[0], self.result_p0[1], self.result_p1[0], self.result_p1[1])
        line_depth = np.array([self.img_medianBlur[p[0], p[1]]
                              for p in line_point])
        print("抓取线", np.mean(line_depth), np.min(line_depth))
        print("抓取点坐标", self.result_center)
        # self.processed_depth_img[int(self.grasp_planer.result_center[0]),int(self.grasp_planer.result_center[1])]
        self.grasp_depth = np.mean(line_depth)#*0.5+np.min(line_depth)*0.5
        print("抓取估计深度为", self.grasp_depth)
        if self.grasp_depth < 0.1:
            print("@@@深度获取异常")
            self.grasp_depth = np.mean(line_depth)  # self.mean
        self.grasp_depth = (self.grasp_depth+self.min_depth)/2
        self.grasp_depth = self.grasp_depth+0.14+0.08  # -0.16#此处为把140的爪子长度进行补偿
        print("抓取最终深度为", self.grasp_depth)
        return self.grasp_depth

    def myline(self, startx, starty, endx, endy):
        line = []
        if abs(endy - starty) > abs(endx - startx):
            if endy > starty:
                for y in range(starty, endy):
                    x = int((y - starty) * (endx - startx) /
                            (endy - starty)) + startx
                    line.append([y, x])
            else:
                for y in range(endy, starty):
                    x = int((y - starty) * (endx - startx) /
                            (endy - starty)) + startx
                    line.append([y, x])
            return line
        if abs(endy - starty) <= abs(endx - startx):
            if endx > startx:
                for x in range(startx, endx):
                    y = int((x - startx) * (endy - starty) /
                            (endx - startx)) + starty
                    line.append([y, x])
            else:
                for x in range(endx, startx):
                    y = int((x - startx) * (endy - starty) /
                            (endx - startx)) + starty
                    line.append([y, x])
            return line

    @classmethod
    def process(cls, image):
        img = image[cls.CROP_START[1]:cls.CROP_START[1] + cls.CROP_SIZE,
                    cls.CROP_START[0]:cls.CROP_START[0] + cls.CROP_SIZE]
        img = cv2.resize(img, (300, 300))
        out_image = cv2.copyMakeBorder(
            img, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)
        # mask = ((out_image > 1.2) | (out_image < 0.1)).astype(np.uint8)
        mask = (out_image < 0.1).astype(np.uint8)
        depth_scale = np.abs(out_image).max()
        out_image = out_image.astype(np.float32) / depth_scale
        # out_image = cv2.inpaint(out_image, mask, 1, cv2.INPAINT_NS)
        out_image = out_image[10:-10, 10:-10]
        out_image = out_image * depth_scale
        return out_image

    @classmethod
    def process_rgb(cls, image):
        img = image[cls.CROP_START[1]:cls.CROP_START[1] + cls.CROP_SIZE,
                    cls.CROP_START[0]:cls.CROP_START[0] + cls.CROP_SIZE]
        img = cv2.resize(img, (300, 300))
        return img

    def process_to_original(self, p):  # 这里是要把在300x300的小图下的抓取点坐标还原到整个摄像头的原始视野中
        zoom = np.array(p) / self.SCALE
        crop = zoom + self.CROP_START
        return crop
