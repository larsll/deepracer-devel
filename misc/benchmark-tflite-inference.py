import json
import time

import cv2
import numpy as np
import tensorflow as tf

def rgb2gray(rgb):
    return np.dot(rgb[..., :3], [0.299, 0.587, 0.114])

def main():

    model_path = 'sample-models/Sample_single_cam'
    model_file = f'{model_path}/model.tflite'
    metadata_file = f'{model_path}/model_metadata.json'

    with open(metadata_file, encoding="UTF-8") as jsonin:
        model_metadata=json.load(jsonin)

    my_sensor = [sensor for sensor in model_metadata['sensor'] if sensor != "LIDAR"][0]
    arr = []

    interpreter = tf.lite.Interpreter(model_path=model_file)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    cam = cv2.VideoCapture(0, apiPreference=cv2.CAP_V4L2)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cam.set(cv2.CAP_PROP_FPS, 30.0)
    ret, img = cam.read()

    start_time = time.time()

    IMAGES = 200
    for i in range(0, IMAGES):

        ret, img = cam.read()
        img = cv2.resize(img, dsize=(160, 120), interpolation=cv2.INTER_CUBIC)
        img_arr = np.array(img)
        img_arr = rgb2gray(img_arr)
        img_arr = np.expand_dims(img_arr, axis=2)

        interpreter.set_tensor(input_details[0]['index'], [img_arr.astype(np.float32)])

        interpreter.invoke()

        # The function `get_tensor()` returns a copy of the tensor data.
        # Use `tensor()` in order to get a pointer to the tensor.
        y_output = interpreter.get_tensor(output_details[0]['index'])
        # print(y_output)

        arr.append (y_output)    

    end_time = time.time()
    elapsed_time = end_time - start_time
    print("--- %s seconds, %s fps ---" % (elapsed_time, IMAGES / elapsed_time))

if __name__ == "__main__": main()

