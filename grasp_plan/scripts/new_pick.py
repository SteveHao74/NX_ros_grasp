
import os
import sys

from requests.sessions import TooManyRedirects
from cv2 import  cv2
import time
import Pyro4
import numpy as np
import tkinter as tk
from matplotlib import pyplot as plt
from servo_grasp import ServoGrasp


file_path = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.abspath(os.path.join(file_path, '..'))
print(root_path)
sys.path.append(root_path)

try:
    # from utils.realsenseL515 import RealsenseSensor
    from utils.realsense_sensor import RealsenseSensor
    from grasp_2d import Grasp2D
    from tf_mapper import TFMapper_EyeInHand
    from camera import CameraBase

except Exception as e:
    print('import utils error')
    raise e

Pyro4.config.SERIALIZER = 'pickle'
grasp_server = Pyro4.Proxy("PYRO:grasp@10.12.120.55:6665")
Pyro4.asyncproxy(grasp_server)

total_length = []
height_min = 0.146#一定要高于这个高度，避免爪子碰到

HHHH = 40
def Plan(image):
    print(image.shape)
    image = image.astype('float32')
    np.save('ddd.npy', image)
    return grasp_server.plan(image, 70)

def  vector_to_matrix(vector):
    T_matrix = np.eye(4)
    r_vector = vector[3:]
    t_vector = vector[:3]
    R_matrix,_ = cv2.Rodrigues(r_vector)
    T_matrix[:3, :3] = R_matrix
    T_matrix[0, 3] = t_vector[0] 
    T_matrix[1,3] = t_vector[1]
    T_matrix[2,3] = t_vector[2]
    return T_matrix

class GrapsPanler(object):
    CROP_START = np.array([400, 200])#200, 50
    CROP_SIZE = 250#480
    SCALE = 300 / CROP_SIZE

    def __init__(self, rgb, depth,table_depth, camera, ur_status=None):
        self.rgb = rgb
        self.camera = camera
        self.ur_status = ur_status
        self.image = self.process(depth)
        # decimation = rs.spatial_filter()
        # self.image = decimation.process(self.image)
        self.min_depth,self.img_medianBlur    = self.get_min_depth()
        self.depth = cv2.GaussianBlur(self.image,(3,3),0)
        self.table_depth = cv2.GaussianBlur(self.process(table_depth),(3,3),0)
        self.mean = np.mean(self.depth)
        self.std = np.std(self.depth)  
        #cor:1.1981683 ; 0.32395524  #mean; std 
        #gmd: 0.7338126;0.00363 ;  #30000: 0.69948566;0.005129744
        #jaq: 1.5008891 ; 0.04099764；#30000: 1.4984636 0.04409859 
        # self.normalize_depth = (self.depth-self.mean*np.ones(self.depth.shape))/self.std  * 0.32395524 +1.1981683*np.ones(self.depth.shape)
        # self.normalize_depth = (self.depth-self.mean*np.ones(self.depth.shape))/self.std  *  0.005129744+0.69948566*np.ones(self.depth.shape)
        # self.normalize_depth = (self.depth-self.mean*np.ones(self.depth.shape))/self.std  * 0.04409859 +1.4984636*np.ones(self.depth.shape)    
        # print("maxmaxmax:",np.max(self.depth))
        # self.grasp_pyro = Plan(self.depth-self.table_depth)#ggrot此处需要减去桌面
        self.grasp_pyro = Plan(self.depth)

    def get_min_depth(self):
        img_GaussianBlur = cv2.GaussianBlur(self.image,(5,5),0)
        img_medianBlur = cv2.medianBlur(img_GaussianBlur,5)#,(5,5))
        a = img_medianBlur[np.nonzero(img_medianBlur)]
        b = np.array([n for n in a if n>0.1])
        min_depth = np.min(b)
        print('最小深度:', min_depth)
        return min_depth,img_medianBlur

    def is_ready(self):
        return self.grasp_pyro.ready

    def get_grasp(self):
        if not self.is_ready():
            return None
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
        # p0, p1, d0, d1, q,pic = result
        p0, p1, d0, d1, q = result
        # plt.clf()
        # plt.axis('off')
        # plt.imshow(pic)
        # plt.show()
        self.result_p0 = p0
        self.result_p1 = p1
        self.result_center =(p0+p1)/2
        p0_temp = self.process_to_original(p0)#这里是要把在300x300的小图下的抓取点坐标还原到整个摄像头的原始视野中
        p1_temp = self.process_to_original(p1)
        self.grasp = Grasp2D.from_endpoint(p0_temp, p1_temp, d0, d1, q)
        return self.grasp

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
        out_image = cv2.inpaint(out_image, mask, 1, cv2.INPAINT_NS)
        out_image = out_image[10:-10, 10:-10]
        out_image = out_image * depth_scale
        return out_image

    @classmethod
    def process_rgb(cls, image):
        img = image[cls.CROP_START[1]:cls.CROP_START[1] + cls.CROP_SIZE,
                    cls.CROP_START[0]:cls.CROP_START[0] + cls.CROP_SIZE]
        img = cv2.resize(img, (300, 300))
        return img

    def process_to_original(self, p):#这里是要把在300x300的小图下的抓取点坐标还原到整个摄像头的原始视野中
        zoom = np.array(p) / self.SCALE
        crop = zoom + self.CROP_START
        return crop


class NewPick(ServoGrasp):
    # put_pose = np.array([[0, 1, 0, 0.4],
    #                      [1, 0, 0, -0.2],
    #                      [0, 0, -1, 0.4],
    #                      [0, 0, 0, 1]])

    def __init__(self, camera, urs, tf):
        self.planing = False
        self.grasp = None
        self.target = None
        self.ex = False
        self.shot_one = False
        self.image_num = 0
        self.init_pose = np.array([-0.50416,-0.16872,0.29645,2.204,-1.725,0.961])
        self.init_pose_matrix = vector_to_matrix(self.init_pose) #使用opencv库将旋转向量表示法转换为旋转矩阵表示法

        urs.movel_to(self.init_pose, v=0.1)
        
        super().__init__(camera, urs, tf)
        


    def video_loop(self):
        success = False
        while not success:
            success, rgb, depth = self.camera.read(5)
        # print("rgb is",rgb)   
        # print("depth is",depth)
        # print("depth max",np.mean(depth))
        if self.planing:
            if self.grasp_planer is None:
                # print("get it ",self.table_depth)
                ur_status = self.urs.ur_status()
                depths = [depth] + [self.camera.read_depth(1) for _ in range(4)]
                print("sha",depths)
                # depths =  [self.camera.read_depth(1) for _ in range(4)]
                depth = np.mean(depths, axis=0)#计算几张深度图矩阵的均值，depth存着的是均值
                
                mean = np.mean(depth)
                new_std = np.std(depth)
                # print("平均值",mean)
                # print("标准差",new_std)
            
                # fig = plt.figure()
                # ax1 = fig.add_subplot(221)
                # ax1.imshow(depth, cmap= plt.cm.gray)
                # depth =  (depth-self.table_depth)#/new_std* 0.005
                # ax2 = fig.add_subplot(222)
                # ax2.imshow(self.table_depth, cmap= plt.cm.gray)
                # ax3 = fig.add_subplot(223)
                # ax3.imshow(depth, cmap= plt.cm.gray)
                # plt.show()

                # print("-------------shahaoshahao",depth)
                self.grasp_planer = GrapsPanler(
                    rgb, depth,self.table_depth, self.camera, ur_status)
                self.min_depth = self.grasp_planer.min_depth
                self.mean= self.grasp_planer.mean
                self.std= self.grasp_planer.std
                self.grasp_depth = self.grasp_planer.min_depth
                self.processed_depth_img = self.grasp_planer.img_medianBlur#得到经过中值滤波的深度图
                self.processed_table_depth = self.grasp_planer.table_depth


            elif self.grasp_planer.is_ready():
                ggg = self.grasp_planer.get_grasp()
                if ggg is None:
                    print('\033[31;41m 规划失败!!!!!! \033[0m')
                self.grasp = ggg
                self.planing = False
                if self.grasp :
                    # depth_point = self.grasp.center - GrapsPanler.CROP_START
                    line_point= self.myline(self.grasp_planer.result_p0[0],self.grasp_planer.result_p0[1],self.grasp_planer.result_p1[0],self.grasp_planer.result_p1[1])
                    line_depth = np.array([self.processed_depth_img[p[0],p[1]]  for  p  in line_point ])
                    print("抓取线",np.mean(line_depth),np.min(line_depth))
                    print("抓取点坐标",self.grasp_planer.result_center)
                    self.grasp_depth=np.mean(line_depth)*0.1+np.min(line_depth)*0.9#self.processed_depth_img[int(self.grasp_planer.result_center[0]),int(self.grasp_planer.result_center[1])] 

                    print("抓取估计深度为",self.grasp_depth)
                    # if self.grasp_depth <0.1:
                    #     print("@@@深度获取异常")
                    #     self.grasp_depth = np.mean(line_depth)#self.mean
                    # self.grasp_depth=(self.grasp_depth+self.min_depth)/2 
                    print("抓取最终深度为",self.grasp_depth)
                    offset = self.processed_table_depth[int(self.grasp_planer.result_center[0]),int(self.grasp_planer.result_center[1])]
                    # self.grasp_depth = self.grasp.depth + offset
                    self.grasp_depth = self.grasp_depth -0.16#此处为把140的爪子长度进行补偿
                    print("抓取最终深度为",self.grasp_depth)

                    self.grasp.depth = self.grasp_depth#把通过工程计算得到的深度直接替换掉来自于网络模型的深度，因为网络模型会给深度为0，那样会使得后续出现NAN，而且模型也无法提供深度的估计

                self.grasp_planer = None


        if self.ex:
            # 如果没有目标或者达到目标则切换下一个目标
            if self.target is None or self.is_get_target():
                if self.targets:
                    self.target = self.targets.pop(0)
                    # a=np.array([0.29964,-0.10738,0.39318,2.9278,1.0482,0.0885])
                    # urs.movel_to(a, v=0.1)
                    self.start_target()
                else:
                    self.target = None
        # save_path="/home/shahao/gmnet_robot_shahao"            
        if self.shot_one:
            np.save('../validation_test/%03d_raw.npy'%self.image_num, depth)
            np.save('../validation_test/%03d.npy'%self.image_num, GrapsPanler.process(depth))
            cv2.imwrite('../validation_test/%03d.png'%self.image_num, GrapsPanler.process_rgb(rgb))
            self.image_num += 1
            self.shot_one = False

        plt.clf()
        plt.axis('off')
        plt.imshow(rgb)
        plt.imshow(GrapsPanler.process(depth))
        # plt.imshow(GrapsPanler.process(depth))
        self.plot_rect(GrapsPanler.CROP_START, GrapsPanler.CROP_SIZE, GrapsPanler.CROP_SIZE)
        # plt.colorbar()
        plt.text(50, 50, 'ON' if self.planing else 'OFF', size=150,
                 color='r', style="italic", weight="light")
        # plt.text(350, 50, 'w:%.4f' % self.get_width(depth), size=150,
        #          color='r', style="italic", weight="light")
        if self.grasp is not None:
            self.plot_grasp(self.grasp)
            plt.text(50, 420, 'current_len:%.4f' % self.grasp.quality, size=160,
                     color='r', style="italic", weight="light")
            total_length.append(self.grasp.quality)
            plt.text(50, 440, 'max_len:%.4f' % max(total_length), size=160,
                     color='g', style="italic", weight="light")
        self.canvas.draw()
        # 30ms后重复执行
        self.root.after(1, self.video_loop)

    def myline(self,startx, starty, endx, endy):
        line = []
        if abs(endy - starty) > abs(endx - startx):
            if endy > starty:
                for y in range(starty, endy):
                    x = int((y - starty) * (endx - startx) / (endy - starty)) + startx
                    line.append([y, x])
            else:
                for y in range(endy, starty):
                    x = int((y - starty) * (endx - startx) / (endy - starty)) + startx
                    line.append([y, x])
            return line
        if abs(endy - starty) <= abs(endx - startx):
            if endx > startx:
                for x in range(startx, endx):
                    y = int((x - startx) * (endy - starty) / (endx - startx)) + starty
                    line.append([y, x])
            else:
                for x in range(endx, startx):
                    y = int((x - startx) * (endy - starty) / (endx - startx)) + starty
                    line.append([y, x])
            return line

    def plot_rect(self, start, width, height):
        plt.plot((start[0], start[0]), (start[1],
                                        start[1]+height), 'r-', linewidth=8)
        plt.plot((start[0]+width, start[0]+width),
                 (start[1], start[1]+height), 'r-', linewidth=8)
        plt.plot((start[0], start[0]+width),
                 (start[1], start[1]), 'r-', linewidth=8)
        plt.plot((start[0], start[0]+width), (start[1] +
                                              height, start[1]+height), 'r-', linewidth=8)

    def plan(self):
        self.planing = True

    def start_target(self):
        if self.target[0] == 'move':
            self.urs.movel_to(self.target[1], v=0.1)
        elif self.target[0] == 'close':
            self.gripper.goto(self.target[1])
            if self.target[1] == 0:
                time.sleep(0.5)
        elif self.target[0] == 'open':
            self.gripper.open()
            time.sleep(1)

    def is_get_target(self):
        if self.target is None:
            return True
        elif self.target[0] == 'move':
            ur_status = self.urs.ur_status()
            if np.sum(np.abs(self.target[1] - ur_status)) < 0.001:
                return True
            else:
                return False
        elif self.target[0] == 'close' or self.target[0] == 'open':
            return True

    def execute(self):
        if self.grasp is not None:
            self.ex = True
            ur_status = self.urs.ur_status()
            print('ur_status',ur_status)

            g_world, width = self.tf.grasp2d_to_matrix(self.grasp, ur_status)#得到了目标抓取在世界坐标系下的位姿，和抓取宽度（两个抓取点间的距离）
            print("世界坐标系下抓取位姿",g_world)
            temp_0 = np.array([g_world[0,0],g_world[1,0],g_world[2,0]])
            temp_1 = np.array([g_world[0,1],g_world[1,1],g_world[2,1]])
            print(temp_0)

            g_world[:3,0] = -temp_1
            g_world[:3,1] = temp_0
                        # g_world[:3,0],g_world[:3,1] = temp_1,temp_0 
            print("改之后",g_world)
            #gp_world = self.tf.matrix_translation(g_world, z=height_min)
            g_base = self.tf.base_from_world(g_world)
            
            # g_base[:3,0],g_base[:3,1] = g_base[:3,1],g_base[:3,0] 
            # g_base[2,3] +=0.175#g_base[2,3]+0.137#0.16#0.157
            # if g_base[2,3] < 0.156:
            #     g_base[2,3] = 0.156
            gtop_world =  g_world.copy()
            self.init_pose_matrix_world = self.tf.base_to_world(self.init_pose_matrix)
            print("初始位子",self.init_pose_matrix)
            print("初始位子——世界",self.init_pose_matrix_world)

            gtop_world[2,3] =  0.8#self.init_pose_matrix_world[2,3] 
            print("世界下top",gtop_world)
            gtop_base = self.tf.base_from_world(gtop_world) 
            targets = []
            print("基座下top",gtop_base)
            print("基座下抓取位姿",g_base)
            print("width",width)
            targets.append(['move', self.init_pose_matrix])
            # targets.append(['move',gtop_base])
            targets.append(['open', None])
            targets.append(['close', width])
            targets.append(['move', g_base])
            targets.append(['close', 0])
            # targets.append(['close', 0.04])
            targets.append(['move',gtop_base])
            targets.append(['move', self.init_pose_matrix])
            targets.append(['open', None])
            self.targets = targets
    
    def shot(self):
        self.shot_one = True

    def add_button(self):
        """ 增加三个按键 """
        button_frame = tk.Frame(self.root)
        button_frame.pack()
        b0 = tk.Button(button_frame, text='Plan', font=('Arial', 12),
                       width=12, height=3, command=self.plan)
        b0.pack(side=tk.LEFT)
        b1 = tk.Button(button_frame, text='Execute', font=('Arial', 12),
                       width=12, height=3, command=self.execute)
        b1.pack(side=tk.LEFT)
        b2 = tk.Button(button_frame, text='Open', font=('Arial', 12),
                       width=12, height=3, command=self.gripper.open)
        b2.pack(side=tk.LEFT)
        b4 = tk.Button(button_frame, text='Shot', font=('Arial', 12),
                       width=12, height=3, command=self.shot)
        b4.pack(side=tk.LEFT)
        b3 = tk.Button(button_frame, text='Exit', font=('Arial', 12),
                       width=12, height=3, command=self.root.quit)
        b3.pack(side=tk.LEFT)   

if __name__ == "__main__":
    # insces_path = os.path.join(root_path, 'cameras/images/primesense')
    insces_path = os.path.join(root_path, 'cameras/images_intrinsic/realsense_D435')
    out_path = os.path.join(root_path, 'cameras/arm_images_extrinsic/realsense_D435')
    # intr = None
    cam2base = np.load(os.path.join(
        root_path, 'cameras/arm_images_extrinsic/realsense_D435/cam2base.npy'))
    # camera = RealsenseSensor(align_to='color',use='color', insces_path=insces_path)
    # camera = PrimesenseSensor(auto_white_balance=True, insces_path=insces_path)

    camera = CameraBase()
    tf = TFMapper_EyeInHand(camera, cam2base)
    # urs = URServo('192.168.1.104', speed=1)
    #TODO urs = 订阅机械臂的末端位姿（基座坐标系下）
    
    UI = NewPick(camera, urs, tf)
    # camera.stop()
    # urs.close()
