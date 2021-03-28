#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from tf import transformations
import pyproj

#q1
# latitude: 49.9000241484
# longitude: 8.90002681746
#q2
# latitude: 49.90001705
# longitude: 8.89997914188
#q4
# latitude: 49.8999839496
# longitude: 8.900031216
#straight
# latitude: 49.900023917
# longitude: 8.90000000857

class mybot:

    def __init__(self):
        self.gps = 0
        self.gps_latitude = 0
        self.gps_longitude = 0
        self.lng_final =  8.90000000857
        self.lat_final = 49.900023917
        self.imu = 0
        self.yaw_initial = 0
        self.fw_azimuth = 0
        self.bw_azimuth = 0
        self.distance = 0

        self.sub_gps = rospy.Subscriber("/fix", NavSatFix, self.show_gps)
        self.sub_imu = rospy.Subscriber("/imu", Imu, self.show_imu) 

    def show_gps(self, coordinates):
        self.gps = coordinates
        self.gps_latitude = coordinates.latitude
        self.gps_longitude = coordinates.longitude
        self.sub_gps.unregister()
        self.pyproj_calc()

    def show_imu(self, angles):
        self.imu = angles
        quaternion = (self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw_initial = euler[2]

    def pyproj_calc(self):
        g = pyproj.Geod(ellps='WGS84')
        (az12, az21, dist) = g.inv(self.gps_longitude, self.gps_latitude, self.lng_final, self.lat_final)
        self.fw_azimuth = az12
        self.bw_azimuth = az21
        self.distance = dist
        self.conversion()
        
    def conversion(self):

        #Convert yaw to 0-360 range
        if self.yaw_initial < 0:
            self.yaw_initial = self.yaw_initial + 360

        #fw_azimuth to 0-360 range
        if self.fw_azimuth < 0:
            self.fw_azimuth = self.fw_azimuth + 360

        print("yaw:", self.yaw_initial)
        print("az12:", self.fw_azimuth)
        print("distance:", self.distance)
        self.linear()

    def rotation(self):
        
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rate = rospy.Rate(10)
        velocity = Twist()

        if(self.fw_azimuth > self.yaw_initial):
            
            if(self.fw_azimuth - self.yaw_initial<180):
                
                while not rospy.is_shutdown():
                
                    while (self.fw_azimuth - self.yaw_initial > 1):
                        velocity.angular.z = 0.2
                        pub.publish(velocity)
                        rospy.Subscriber("/imu", Imu, self.show_imu)
                        break
                        
                    velocity.angular.z = 0
                    pub.publish(velocity)
                    rate.sleep()
            else:
                
                while not rospy.is_shutdown():

                    while (self.yaw_initial - self.fw_azimuth > 1):
                        velocity.angular.z = 0.2
                        pub.publish(velocity)
                        rospy.Subscriber("/imu", Imu, self.show_imu)
                        break
                        

                    velocity.angular.z = 0
                    pub.publish(velocity)
                    rate.sleep()

        if(self.yaw_initial > self.fw_azimuth):
           
            if(self.yaw_initial - self.fw_azimuth < 180):
                
                while not rospy.is_shutdown():
                   
                    while (self.yaw_initial - self.fw_azimuth > 1):
                        velocity.angular.z = -0.2
                        pub.publish(velocity)
                        rospy.Subscriber("/imu", Imu, self.show_imu)
                        break
                        

                    velocity.angular.z = 0
                    pub.publish(velocity)
                    rate.sleep()

            else:
       
                while not rospy.is_shutdown():
                    
                    while (self.fw_azimuth - self.yaw_initial > 1):
                        velocity.angular.z = 0.2
                        pub.publish(velocity)
                        rospy.Subscriber("/imu", Imu, self.show_imu)
                        break

                    velocity.angular.z = 0
                    pub.publish(velocity)
                    rate.sleep()
                    self.linear()
    

    def linear(self):
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rate = rospy.Rate(1000000)
        velocity = Twist()

        while self.distance > 0.8:
            velocity.linear.x = 1
            pub.publish(velocity)
            rospy.Subscriber("/fix", NavSatFix, self.show_gps)
            break

        velocity.angular.z = 0
        pub.publish(velocity)
        rate.sleep()


if __name__ == "__main__":
    
    rospy.init_node("ATT", anonymous=True)
    gps = mybot()
    rospy.spin()