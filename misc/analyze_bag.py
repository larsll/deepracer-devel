#/usr/bin/python3

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from rclpy.time import Time

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]



if __name__ == "__main__":

        bag_file = 'output/rosbag2_2021_09_27-15_24_32/rosbag2_2021_09_27-15_24_32_0.db3'

        parser = BagFileParser(bag_file)
        prev_time = 0
        max_diff = 0
        time = 0
        diff = 0.0
        msgs = parser.get_messages("/video/sensor_stream")

        for m in msgs:
            timestamp = m[0]
            cameramsg = m[1]
            #print (cameramsg.images[0].header.stamp)
            prev_time = time
            time = Time(seconds=cameramsg.images[0].header.stamp.sec, nanoseconds=cameramsg.images[0].header.stamp.nanosec).nanoseconds/1e9
            if prev_time > 0:
                diff = time - prev_time
            max_diff = max(diff, max_diff)
            print ("{:.2f}".format(diff))
        
        print("Max: {}".format(max_diff))