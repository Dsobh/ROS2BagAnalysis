import yaml
import sys
from datetime import datetime

if len(sys.argv) == 2:
	bag_path = sys.argv[1]
else:
	exit()
	
with open(bag_path + 'metadata.yaml', 'r') as file:
	bag_file = yaml.safe_load(file)
	
duration_ns = bag_file['rosbag2_bagfile_information']['duration']['nanoseconds']
#duration = duration_ns / 1e9

files = bag_file['rosbag2_bagfile_information']['files']
topics = bag_file['rosbag2_bagfile_information']['topics_with_message_count']

total_duration = 0
total_messages = 0

for file_entry in files:
	total_duration += file_entry['duration']['nanoseconds']
	total_messages += file_entry['message_count']
	
print("Total messages: " + str(total_messages))
print("Total Duration: " + str(total_duration / 1e9))

for topic in topics:
	print("Topic " + topic['topic_metadata']['name'] + ' has a message count of ' + str(topic['message_count']))
	

