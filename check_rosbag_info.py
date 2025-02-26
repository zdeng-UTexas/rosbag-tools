# import pyrosbag

# bag = pyrosbag.Bag('/robodata/ARL_SARA/GQ-dataset/bagfiles/2024-08-12-11-32-46.bag')

# print(dir(bag))

# # List topics and message counts
# info = bag.get_type_and_topic_info()
# for topic, topic_info in info.topics.items():
#     print(f"Topic: {topic}, Messages: {topic_info.message_count}, Type: {topic_info.msg_type}")

import rosbag

bag_path = '/robodata/ARL_SARA/GQ-dataset/bagfiles/2024-08-13-12-05-41.bag'

# Open the bag file
with rosbag.Bag(bag_path, 'r') as bag:
    # Get topics and types
    info = bag.get_type_and_topic_info()
    for topic, topic_info in info.topics.items():
        print(f"Topic: {topic}, Messages: {topic_info.message_count}, Type: {topic_info.msg_type}")
