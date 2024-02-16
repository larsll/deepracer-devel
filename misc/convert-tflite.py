import tensorflow.compat.v1 as tf

converter = tf.lite.TFLiteConverter.from_frozen_graph(
    graph_def_file='sample-models/Sample_single_cam/model.pb',
    input_shapes = {'main_level/agent/main/online/network_0/observation/observation':[1,120,160,1]},
    input_arrays = ['main_level/agent/main/online/network_0/observation/observation'],
    output_arrays = ['main_level/agent/main/online/network_1/ppo_head_0/policy']
)
tflite_model = converter.convert()

with open('sample-models/Sample_single_cam/model.tflite', 'wb') as f:
  f.write(tflite_model)