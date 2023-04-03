import tf2_ros
import rosbag
import rospy
from rosbag import Bag, ROSBagException
from tqdm import tqdm
import matplotlib.pyplot as plt 

def load_buffer(path):
    tf_topics = ['/tf', '/tf_static']
    buffer = tf2_ros.Buffer(rospy.Duration(3600.0 * 3600.0))
    try:
        with Bag(path, 'r') as bag:
            for topic, msg, stamp in tqdm(bag.read_messages(topics=tf_topics),
                                            desc='%s: reading transforms' % path.split('/')[-1],
                                            total=bag.get_message_count(topic_filters=tf_topics)):
                if topic == '/tf':
                    for tf in msg.transforms:
                        buffer.set_transform(tf, 'bag')
                elif topic == '/tf_static':
                    for tf in msg.transforms:
                        buffer.set_transform_static(tf, 'bag')
    except ROSBagException as ex:
        print('Could not read %s: %s' % (path, ex))
    return buffer


if __name__ == "__main__":
    coordinates_x = []
    coordinates_y = []
    rospy.init_node('tf_listener')
    path = '/home/robert/bagfiles/husky_2022-09-23-12-38-31.bag'
    bag = rosbag.Bag('/home/robert/bagfiles/husky_2022-09-23-12-38-31.bag')
    start_time = bag.get_start_time()
    buffer = load_buffer(path)
    for topic, msg, time in bag.read_messages(topics=['/camera_front/image_color/compressed']):
        time = rospy.Time.from_sec(time.to_sec())
        time_sec = time.to_sec()
        start_time = bag.get_start_time()
        time_sec = time_sec - start_time
        print(time_sec)
        if(int(time_sec) % 5 == 0):
            try:
                transform = buffer.lookup_transform_full("map",time, "base_link",time, "map", rospy.Duration(1.0))
                coordinates_x.append(transform.transform.translation.x)
                coordinates_y.append(transform.transform.translation.y)
            except tf2_ros.ExtrapolationException as ex:
                continue 
    plt.plot(coordinates_x, coordinates_y)
    plt.show()
