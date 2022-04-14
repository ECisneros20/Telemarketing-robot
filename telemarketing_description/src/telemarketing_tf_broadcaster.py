#!/usr/bin/env python
import tf
import rospy

if __name__ == '__main__':

  x_cwheel=0.0240
  y_cwheel=0.0000
  z_cwheel=-0.0420

  x_dwheel=0.0000
  y_dwheel=0.175
  z_dwheel=0.0525

  rospy.init_node('telemarketing_tf_broadcaster')
  
  # blink is a contraction of base link
  blink_b = tf.TransformBroadcaster()
  blink_bfprint = tf.TransformBroadcaster()
  rdrivelink_blink = tf.TransformBroadcaster()
  ldrivelink_blink = tf.TransformBroadcaster()
  frcasterlink_blink = tf.TransformBroadcaster()
  flcasterlink_blink = tf.TransformBroadcaster()
  brcasterlink_blink = tf.TransformBroadcaster()
  blcasterlink_blink = tf.TransformBroadcaster()
  


  bcamera_blink = tf.TransformBroadcaster()
  brplidar_blink = tf.TransformBroadcaster()
  bfcaster_blink = tf.TransformBroadcaster()
  bbcaster_blink = tf.TransformBroadcaster()

  #rate = rospy.Rate(100)

  while not rospy.is_shutdown():

    blink_b.sendTransform((0.00, 0.00, 0.00),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),"base_link","base")

    blink_bfprint.sendTransform((0.00, 0.00, 0.00),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),"base","base_footprint")

    rdrivelink_blink.sendTransform((-0.026, -0.174, 0.661),
        tf.transformations.quaternion_from_euler(1.5708, 0, 0),
        rospy.Time.now(),"r_dwheel","base_link")

    ldrivelink_blink.sendTransform((-0.007, 0.169, 0.661),
        tf.transformations.quaternion_from_euler(1.5708, 0, 0),
        rospy.Time.now(),"l_dwheel","base_link")

    frcasterlink_blink.sendTransform((0.165, -0.186, 0.609),
        tf.transformations.quaternion_from_euler(1.5708, 0, 0),
        rospy.Time.now(),"fr_cwheel","base_link")

    flcasterlink_blink.sendTransform((0.165, 0.181, 0.609),
        tf.transformations.quaternion_from_euler(1.5708, 0, 0),
        rospy.Time.now(),"fl_cwheel","base_link")

    brcasterlink_blink.sendTransform((-0.199, -0.186, 0.609),
        tf.transformations.quaternion_from_euler(1.5708, 0, 0),
        rospy.Time.now(),"br_cwheel","base_link")
    
    blcasterlink_blink.sendTransform((-0.199, 0.181, 0.609),
        tf.transformations.quaternion_from_euler(1.5708, 0, 0),
        rospy.Time.now(),"bl_cwheel","base_link")


'''
    bcamera_blink.sendTransform((x_dwheel, y_dwheel, z_dwheel),
        tf.transformations.quaternion_from_euler(1.5708, 0, 0),
        rospy.Time.now(),"l_dwheel","base_link")

    bcamera_blink.sendTransform((x_cwheel, y_cwheel, z_cwheel),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),"fr_cwheel","fr_scwheel")

    bcamera_blink.sendTransform((x_cwheel, y_cwheel, z_cwheel),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),"fl_cwheel","fl_scwheel")

    bcamera_blink.sendTransform((x_cwheel, y_cwheel, z_cwheel),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),"br_cwheel","br_scwheel")

    bcamera_blink.sendTransform((x_cwheel, y_cwheel, z_cwheel),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),"bl_cwheel","bl_scwheel")

'''