import carla
from carla import ColorConverter as cc
from multiprocessing import Process
from multiprocessing import Queue
import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import numpy as np


from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, QThread, pyqtSignal

import numpy as np
import sys

from PyQt5.QtCore import QThread




sensor_queue = Queue()


def pyqt_show():

    class ImageGenerator(QThread):
        image_generated = pyqtSignal(np.ndarray)

        def run(self):
            while True:
                # Generate a new random image
                img_source = sensor_queue.get(True, 1.0)
                img = img_source[1]

                #img = np.random.randint(0, 256, (500, 500, 3), dtype=np.uint8)

                # Emit a signal with the image
                self.image_generated.emit(img)

                # Sleep for a bit to simulate work
                self.msleep(100)

    class ImageFlowWidget(QWidget):
        def __init__(self):
            super().__init__()

            self.image_label = QLabel(self)

            layout = QVBoxLayout()
            layout.addWidget(self.image_label)
            self.setLayout(layout)

            self.image_generator = ImageGenerator()
            self.image_generator.image_generated.connect(self.update_image)
            self.image_generator.start()

        def update_image(self, img):
            # Convert the image to a QImage
            qimg = QImage(img.data, img.shape[1], img.shape[0], img.strides[0], QImage.Format_RGB888)

            # Convert the QImage to a QPixmap and show it
            self.image_label.setPixmap(QPixmap.fromImage(qimg))

    app = QApplication(sys.argv)
    widget = ImageFlowWidget()
    widget.show()
    sys.exit(app.exec_())


def carla_data():
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


        camera_init_trans = carla.Transform(carla.Location(z=3.5))
        camera_bp = sim_world.get_blueprint_library().find('sensor.camera.rgb')
        camera = sim_world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

        def sensor_callback(sensor_data):
            sensor_data.convert(cc.Raw)
            array = np.frombuffer(sensor_data.raw_data, dtype=np.dtype("uint8"))
            # image is rgba format
            array = np.reshape(array, (sensor_data.height, sensor_data.width, 4))
            print(sensor_data.height)
            print(sensor_data.width)
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            sensor_queue.put((sensor_data.frame, array))

        #camera.listen(lambda image: image.save_to_disk('out/%06d.png' % image.frame))
        camera.listen(lambda image: sensor_callback(image))

        # set the vehicle autopilot mode
        ego_vehicle.set_autopilot(True)

        while True:
            sim_world.wait_for_tick()
            spectator = sim_world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(x=-20,y=0,z=8),
                                                    carla.Rotation(pitch=0,yaw=0,roll=0)))
            # s_frame = sensor_queue.get(True,2.0)
            # print(s_frame[0])

    finally:
        camera.destroy()
        ego_vehicle.destroy()


def main():
    p1 = Process(target=pyqt_show, args=())
    p2 = Process(target=carla_data, args=())
    p1.start()
    p2.start()
    p1.join()
    p2.join()

if __name__ == '__main__':

    main()