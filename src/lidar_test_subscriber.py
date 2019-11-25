'''
This is a test subscriber node that displays some basic metrics based on the laserscan array it receives. 

'''


import rospy
from sensor_msgs.msg import LaserScan
import math

def scan_callback(scan_data):

    #Find the minimum range
    min_value, min_index = min_range_index(data.ranges)
    print("The minimun range value is: ", min_value)
    print("The minimun range index is: ", min_index)

    max_value, max_index = max_range_index(data.ranges)
    print("The max range value is: ", max_value)
    print("the max range index is: ", max_index)

    average_value = average_range(data.ranges)
    print("The average range value is: ", average value)

    average2 = average between indices(data.ranges, 2, 7)
    print("The average between 2 indices is: ", average2)

#return min range
def min_range_index(ranges):

    return (min(ranges), ranges.index(min(ranges)))

#return max range
def max_ranges_index(ranges):

    ranges = [x for x in ranges if not math.isnan(x)]
    return (max(ranges), ranges.index(max(ranges)))

#find the average range
def average_range(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    return(sum(ranges) / float(len(ranges)))

def average_between_indices(ranges, i, j):
    ranges = [x for x in ranges if not math.isnan(x)]
    slice_of_array = ranges[i: j+1]
    return (sum(slice_of_array) / float(len(slice_of_array)))


if __name__ == '__main__':

    #init new node and give it a name
    rospy.init_node('scan_node', anonymous=True)
    #subscribe to topic scan
    rospy.Subscriber("scan", LaserScan, scan_callback)

    #rospy spin
    rospy.spin()