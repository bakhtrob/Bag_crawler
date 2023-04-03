import rosbag
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
import matplotlib.pyplot as plt
from ros_numpy import numpify
import open3d as o3d
from numpy.lib.recfunctions import structured_to_unstructured
import cv2
from nav_msgs.msg import Odometry 
import matplotlib.pyplot as plt 
import matplotlib.animation as animation


def slots(msg):
    return [getattr(msg, var) for var in msg.__slots__] #Возвращает атрибуты объекта класса, какого ??? Bag ? 

def show_cloud(bag_file_name):
    bag = rosbag.Bag(bag_file_name) #Создаем объект класса BAG в из модуля rosbag. 
    #print(type(bag.read_messages()))
    for topic, msg, t in bag.read_messages(topics=['/points']): #Бег файл это формат файла в РОС, для хранения сообщений. 
        #Функция read_message прочитывает сообщение определенного формата в данном случае points. 
        #Так мы получим массив с координатами. 
        #msg.data - масив данных. 
        msg = PointCloud2(*slots(msg))#
        cloud = numpify(msg) #Перевод сообщения в numPy массив.
        if cloud.ndim == 2: #ndim возвращает размеры массивы, в данном случае 2. 
            cloud = cloud.reshape((-1,)) #Делаем из двумерного массив одномерный массив. 

        points = structured_to_unstructured(cloud[['x', 'y', 'z']]) # Берем только координаты. !Выходом двумерный массив! размерности n x 3, где n это число точек в облаке.
        #Каждый подмассив это три координаты определенной точки. 
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)#Используются для визуализации.
        # pcd.colors = o3d.utility.Vector3dVector(np.zeros_like(points) + np.array([0.2, 0.3, 0.5]))
        o3d.visualization.draw_geometries([pcd])
        break
    
    bag.close()

def show_image(bag_file_name):
    bag = rosbag.Bag(bag_file_name)
    for topic, msg, t in bag.read_messages(topics=['/camera_front/image_color/compressed']):

        msg = CompressedImage(*slots(msg))#Создаем объект класса. 

        np_arr = np.fromstring(msg.data, np.uint8)#Новый одномерный массив, инициализированный из текстовых данных в строке.
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)#Преобразовываем в формат изображения. Потому что изначально изображение сжатое, поэтому его надо разжать.
        # plt.imshow(image[..., (2, 1, 0)])  # BGR -> RGB потому что две программы используют обратные типы. Поэтому нужно вернуть в изначальный формат.
        #plt.show()
        print(image)
        cv2.imshow('Image', image) # H x W x 3 Выходом изображение в виде многомерного массива.
        cv2.waitKey(0)

        break
    bag.close()

def show_graph(bag_file_name):
    bag = rosbag.Bag(bag_file_name)
    coordinates_x = []
    coordinates_y = []
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            if(transform.header.frame_id == "map" and transform.child_frame_id == "odom"):
                coordinates_x.append(transform.transform.translation.x)
                coordinates_y.append(transform.transform.translation.y)
                
        
        
    print(coordinates_x, coordinates_y) 
    plt.plot(coordinates_x, coordinates_y)
    plt.show()



if __name__ == '__main__':
    show_graph(bag_file_name='/home/robert/bagfiles/husky_2022-09-23-12-38-31.bag')
