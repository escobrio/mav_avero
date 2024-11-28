import rospy
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
import numpy as np
import tf.transformations as tf_trans
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def plot_three_vectors(v1, v2, v3):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot three vectors
    origin = np.array([0, 0, 0])
    ax.quiver(*origin, *v1, color='r', label='Vector 1')
    ax.quiver(*origin, *v2, color='g', label='Vector 2')
    ax.quiver(*origin, *v3, color='b', label='Vector 3')

    # Set labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    plt.show()

# Example usage:
vector1 = np.array([1, 2, 3])
vector2 = np.array([-2, 0, 4])
vector3 = np.array([3, -1, 2])

plot_three_vectors(vector1, vector2, vector3)


def orientation_callback(quatr, axis):
    quatr_inv = [quatr[0], -quatr[1], -quatr[2], -quatr[3]]
    rot1 = Rotation.from_quat(quatr)
    rot_2 = Rotation.from_quat(quatr_inv)
    result = rot1*axis*rot_2
    return result

def visualizer_imu():
    node_name = 'imu_data_visualization'
    topic_name = '/imu/data'
    rospy.init_node(node_name, anonymous=True) 
    vis_subscriber = rospy.Subscriber(topic_name, Imu, callback)
    
def callback(msg):
    topic_name = 'vector_marker' 

    vector_publisher = rospy.Publisher(topic_name, Marker, queue_size=10)
    rate = rospy.Rate(200)  # 1 Hz

    while not rospy.is_shutdown():
        
        imu_quaternion = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        x = np.array([1, 0, 0])
        y = np.array([0, 1, 0])
        z = np.array([0, 0, 1])
        I_v_x_B = orientation_callback(imu_quaternion, x) 
        I_v_y_B = orientation_callback(imu_quaternion, y) 
        I_v_z_B = orientation_callback(imu_quaternion, z)
        rate.sleep()

if __name__ == '__main__':
    try:
        visualizer_imu()
    except rospy.ROSInterruptException:
        pass
