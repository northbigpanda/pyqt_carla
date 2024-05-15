import carla
from carla import ColorConverter as cc
import argparse
import collections
import datetime
import logging
import math
import random
import numpy as np
import sys
from multiprocessing import Process
import socket
import cv2
import time
import pickle
import struct
import threading



def sensor_callback(sensor_data, conn):
    sensor_data.convert(cc.Raw)
    array = np.frombuffer(sensor_data.raw_data, dtype=np.dtype("uint8"))
    # image is rgba format
    array = np.reshape(array, (sensor_data.height, sensor_data.width, 4))
    # array = np.reshape(array, (600, 800, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]

    array_bytes = pickle.dumps(array)
    array_length = struct.pack('!I', len(array_bytes))
    conn.sendall(array_length+array_bytes)

def lidar_callback(image,conn):
    lidar_range = 50
    # window_size = (408, 361)
    window_size = (361, 408)
    points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0]/4), 4))
    lidar_data = np.array(points[:,:2])
    lidar_data *= min(window_size)/(2.0*(lidar_range))
    lidar_data += (0.5 * window_size[0], 0.5 * window_size[1])
    lidar_data = np.fabs(lidar_data)
    lidar_data = lidar_data.astype(np.int32)
    lidar_data = np.reshape(lidar_data,(-1,2))
    lidar_img_size = (window_size[0], window_size[1], 3)
    lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
    lidar_img[tuple(lidar_data.T)] = (255,255,255)

    array_bytes_1 = pickle.dumps(lidar_img)
    array_length_1 = struct.pack('!I', len(array_bytes_1))
    conn.sendall(array_length_1+array_bytes_1)


def handle_client(client_socket, sensor):
    if sensor.type_id == 'sensor.camera.rgb':
        sensor.listen(lambda image: sensor_callback(image, client_socket))
    elif sensor.type_id == 'sensor.lidar.ray_cast':
        sensor.listen(lambda image: lidar_callback(image, client_socket))




class server(object):
    def __init__(self, ip, port, sensor_list):
        super().__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        self.sock.bind((ip, port))
        self.sock.listen(6)
        print("Server started. Waiting for connections...")

        for sensor in sensor_list:
            self.client, addr = self.sock.accept()
            print("Connection from", addr)
            client_handler = threading.Thread(target=handle_client, args=(self.client,sensor))
            client_handler.start()
            



def main():

    try:
        client = carla.Client('localhost',2000)
        client.set_timeout(2000.0)
        sim_world = client.get_world()

        # settings = sim_world.get_settings()
        # settings.synchronous_mode = True
        # settings.fixed_delta_seconds = 0.05
        # sim_world.apply_settings(settings)

        vehicle_blueprints = sim_world.get_blueprint_library().filter('*vehicle*')
        # get a random valid occupation in the world
        transform = random.choice(sim_world.get_map().get_spawn_points())
        # spawn the vehilce
        ego_vehicle = sim_world.spawn_actor(random.choice(vehicle_blueprints), transform)

        sensor_list = []

        camera_init_trans = carla.Transform(carla.Location(z=3.5))
        window_size = (408, 361)
        camera_bp = sim_world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(window_size[0]))
        camera_bp.set_attribute('image_size_y', str(window_size[1]))
        camera = sim_world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

        sensor_list.append(camera)        

        lidar_range = 50
        lidar_bp_trans = carla.Transform(carla.Location(z=4.5))
        lidar_bp = sim_world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar = sim_world.spawn_actor(lidar_bp,lidar_bp_trans,attach_to=ego_vehicle)

        sensor_list.append(lidar)

        ego_vehicle.set_autopilot(True)

        carlabackend = server('127.0.0.1', 23149, sensor_list)

        while True:
            sim_world.wait_for_tick()
            spectator = sim_world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(x=-20,y=0,z=8),
                                                    carla.Rotation(pitch=0,yaw=0,roll=0)))


    finally:
        carlabackend.client.close()
        carlabackend.sock.close()

if __name__ == '__main__':
    main()