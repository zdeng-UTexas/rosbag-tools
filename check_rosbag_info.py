import pyrosbag

bag = pyrosbag.Bag('/robodata/ARL_SARA/GQ-dataset/bagfiles/2024-08-12-11-32-46.bag')

# List topics and message counts
info = bag.get_type_and_topic_info()
for topic, topic_info in info.topics.items():
    print(f"Topic: {topic}, Messages: {topic_info.message_count}, Type: {topic_info.msg_type}")