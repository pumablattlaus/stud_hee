from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, Twist, Pose

class myPoint(Point):
    def __init__(self, pos=(0,0,0)):
        super(myPoint, self).__init__(*pos)
        
    def __add__(self, p2):
        p_out = Point()
        p_out.x = self.x + p2.x
        p_out.y = self.y + p2.y
        p_out.z = self.z + p2.z
        
        return p_out
    
    def __sub__(self, p2):
        p_out = Point()
        p_out.x = self.x - p2.x
        p_out.y = self.y - p2.y
        p_out.z = self.z - p2.z
        
        return p_out
    

class myPose(Pose):
    def __init__(self, pos=(0,0,0), quatern = (0, 0, 0, 1)):
        point = myPoint(pos)
        orient = Quaternion(*quatern)
        super(myPose, self).__init__(point, orient)
    

if __name__ == '__main__':
    p1 = myPoint((1,2,3))
    p2 = myPoint((1,2,3))
    
    p3 = p1+p2
    
    print(p3)
    
    pose1 = myPose((1,2,3))
    pose2 = myPose((1,2,3))
    
    pose3 = myPose()
    
    pose3.position = pose1.position + pose2.position
    
    print(pose3)