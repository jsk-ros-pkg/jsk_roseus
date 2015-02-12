#!/usr/bin/env python

import rospy
import std_msgs

def send_msgs():
    pubs = { rospy.Publisher('bool', std_msgs.msg.Bool, queue_size=10)  : std_msgs.msg.Bool(data=True),
             rospy.Publisher('byte', std_msgs.msg.Byte, queue_size=10)       : std_msgs.msg.Byte(1),
             rospy.Publisher('bytemultiarray', std_msgs.msg.ByteMultiArray, queue_size=10)       : std_msgs.msg.ByteMultiArray(data=[-1,2,-3]),
             rospy.Publisher('char', std_msgs.msg.Char, queue_size=10)       : std_msgs.msg.Char(2),
             rospy.Publisher('colorrgba', std_msgs.msg.ColorRGBA, queue_size=10)       : std_msgs.msg.ColorRGBA(0.1,0.2,0.3,0.4),
             rospy.Publisher('duration', std_msgs.msg.Duration, queue_size=10)       : std_msgs.msg.Duration(rospy.Time(1.234)),
             rospy.Publisher('empty', std_msgs.msg.Empty, queue_size=10)       : std_msgs.msg.Empty(),
             rospy.Publisher('float32', std_msgs.msg.Float32, queue_size=10)       : std_msgs.msg.Float32(1.234),
             rospy.Publisher('float64', std_msgs.msg.Float64, queue_size=10)       : std_msgs.msg.Float64(5.678),
             rospy.Publisher('header', std_msgs.msg.Header, queue_size=10)       : std_msgs.msg.Header(seq=1234,stamp=rospy.Time(1.234),frame_id="frame"),
             rospy.Publisher('int16', std_msgs.msg.Int16, queue_size=10)       : std_msgs.msg.Int16(-1),
             rospy.Publisher('int32', std_msgs.msg.Int32, queue_size=10)       : std_msgs.msg.Int32(-2),
             rospy.Publisher('int64', std_msgs.msg.Int64, queue_size=10)       : std_msgs.msg.Int64(-3),
             rospy.Publisher('int8', std_msgs.msg.Int8, queue_size=10)       : std_msgs.msg.Int8(-4),
             # rospy.Publisher('multiarraydimension', std_msgs.msg.MultiArrayDimension, queue_size=10)       : std_msgs.msg.,
             # rospy.Publisher('multiarraylayout', std_msgs.msg.MultiAarrayLayout, queue_size=10)       : std_msgs.msg.,
             rospy.Publisher('string', std_msgs.msg.String, queue_size=10)       : std_msgs.msg.String("euslisp"),
             rospy.Publisher('time', std_msgs.msg.Time, queue_size=10)       : std_msgs.msg.Time(rospy.Time(1.234)),
             rospy.Publisher('uint16', std_msgs.msg.UInt16, queue_size=10)       : std_msgs.msg.UInt16(1),
             rospy.Publisher('uint32', std_msgs.msg.UInt32, queue_size=10)       : std_msgs.msg.UInt32(2),
             rospy.Publisher('uint64', std_msgs.msg.UInt64, queue_size=10)       : std_msgs.msg.UInt64(3),
             rospy.Publisher('uint8', std_msgs.msg.UInt8, queue_size=10)       : std_msgs.msg.UInt8(4),
             rospy.Publisher('float32multiarray', std_msgs.msg.Float32MultiArray, queue_size=10)       : std_msgs.msg.Float32MultiArray(data=[1.234,5.678]),
             rospy.Publisher('float64multiarray', std_msgs.msg.Float64MultiArray, queue_size=10)       : std_msgs.msg.Float64MultiArray(data=[1.234,5.678]),
             rospy.Publisher('int16multiarray', std_msgs.msg.Int16MultiArray, queue_size=10)       : std_msgs.msg.Int16MultiArray(data=[-1,2,-3]),
             rospy.Publisher('int32multiarray', std_msgs.msg.Int32MultiArray, queue_size=10)       : std_msgs.msg.Int32MultiArray(data=[-4,5,-6]),
             rospy.Publisher('int64multiarray', std_msgs.msg.Int64MultiArray, queue_size=10)       : std_msgs.msg.Int64MultiArray(data=[-7,8,-9]),
             rospy.Publisher('int8multiarray', std_msgs.msg.Int8MultiArray, queue_size=10)       : std_msgs.msg.Int8MultiArray(data=[-10,11,-12]),
             rospy.Publisher('uint16multiarray', std_msgs.msg.UInt16MultiArray, queue_size=10)       : std_msgs.msg.UInt16MultiArray(data=[1,2,3]),
             rospy.Publisher('uint32multiarray', std_msgs.msg.UInt32MultiArray, queue_size=10)       : std_msgs.msg.UInt32MultiArray(data=[4,5,6]),
             rospy.Publisher('uint64multiarray', std_msgs.msg.UInt64MultiArray, queue_size=10)       : std_msgs.msg.UInt64MultiArray(data=[7,8,9]),
             rospy.Publisher('uint8multiarray', std_msgs.msg.UInt8MultiArray, queue_size=10)       : std_msgs.msg.UInt8MultiArray(data=[10,11,12]),
            }
    rospy.init_node('test_geneus_send_msgs', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        for p,i in pubs.iteritems():
            p.publish(i)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_msgs()
    except rospy.ROSInterruptException:
        pass
