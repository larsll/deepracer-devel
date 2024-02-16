import json
import time

import cv2
import numpy as np
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()

from tensorflow.compat.v1.io.gfile import GFile


def load_session(pb_path, sensor='FRONT_FACING_CAMERA', log_device_placement=True):
    sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True,
                                            log_device_placement=log_device_placement))
    print("load graph:", pb_path)
    with GFile(pb_path, 'rb') as f:
        graph_def = tf.GraphDef()
    graph_def.ParseFromString(f.read())
    sess.graph.as_default()
    tf.import_graph_def(graph_def, name='')
    graph_nodes = [n for n in graph_def.node]
    names = []
    for t in graph_nodes:
        names.append(t.name)

    print(names)
    # For front cameras/stereo camera use the below
    x = sess.graph.get_tensor_by_name(
        f'main_level/agent/main/online/network_0/{sensor}/{sensor}:0')
    y = sess.graph.get_tensor_by_name('main_level/agent/main/online/network_1/ppo_head_0/policy:0')

    return sess, x, y


def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.299, 0.587, 0.114])

def main():

    model_path = 'sample-models/Sample_single_cam'
    model_file = f'{model_path}/model.pb'
    metadata_file = f'{model_path}/model_metadata.json'

    with open(metadata_file, encoding="UTF-8") as jsonin:
        model_metadata=json.load(jsonin)

    my_sensor = [sensor for sensor in model_metadata['sensor'] if sensor != "LIDAR"][0]

    model, obs, model_out = load_session(model_file, my_sensor, False)
    arr = []

    cam = cv2.VideoCapture(0, apiPreference=cv2.CAP_V4L2)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cam.set(cv2.CAP_PROP_FPS, 15.0)
    ret, img = cam.read()

    start_time = time.time()

    IMAGES = 200
    for i in range(0, IMAGES):

        ret, img = cam.read()
        img = cv2.resize(img, dsize=(160, 120), interpolation=cv2.INTER_CUBIC)
        img_arr = np.array(img)
        img_arr = rgb2gray(img_arr)
        img_arr = np.expand_dims(img_arr, axis=2)

        y_output = model.run(model_out, feed_dict={obs:[img_arr]})[0]
        arr.append (y_output)    

    end_time = time.time()
    elapsed_time = end_time - start_time
    print("--- %s seconds, %s fps ---" % (elapsed_time, IMAGES / elapsed_time))

if __name__ == "__main__": main()