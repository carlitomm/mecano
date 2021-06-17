class fuzy_ranmdom_navigation:

    def __init__(self,left_distance, rigth_distance, front_distance):
        self.left_distance = left_distance
        self.rigth_distance = rigth_distance
        self.front_distance = front_distance
      
        """
        self.UcrashLeft = 0
        self.UcrashRigth = 0
        self.UcrashFront = 0
        
        self.UcloseLeft = 0
        self.UcloseRigth = 0
        self.UcloseFront = 0
             
        self.UfarLeft = 0
        self.UfarRigth = 0
        self.UfarFront = 0

        self.Uturnleft = 0
        self.Uforwardleft = 0

        self.Uforward = 0

        self.Uforwardrigth = 0
        self.Uturnrigth = 0

        self.out = 0
        """
        
    def fuzzification(self, left_distance, rigth_distance, front_distance):

        self.UcrashLeft = self.crash_evaluation(left_distance)
        self.UcrashRigth = self.crash_evaluation (rigth_distance)
        self.UcrashFront = self.crash_evaluation(front_distance)
        
        self.UcloseLeft = self.close_evaluation(left_distance)
        self.UcloseRigth = self.close_evaluation(rigth_distance)
        self.UcloseFront = self.close_evaluation(front_distance)
        
        self.UfarLeft = self.far_evaluation(left_distance)
        self.UfarRigth = self.far_evaluation(rigth_distance)
        self.UfarFront = self.far_evaluation(front_distance)

        """
        print ("left is %f"  %self.UcrashLeft)
        print ("right is %f" %self.UcloseLeft )
        print ("front is %f" %self.UfarLeft )
        """

    def Infernece_machine (self):

        self.Uturnleft = (self.UcrashRigth * self.UfarLeft * self.UcrashFront) + (self.UcrashRigth * self.UcloseLeft * self.UcrashFront) + (self.UfarLeft * self.UcloseRigth * self.UcrashFront) + (self.UfarLeft * self.UcloseRigth * self.UfarFront)
        
        self.Uforwardleft = (self.UcloseRigth * self.UfarLeft * self.UcloseFront) + (self.UfarLeft * self.UcrashRigth * self.UcloseFront) + (self.UfarLeft * self.UcrashRigth * self.UfarFront) + (self.UfarLeft * self.UfarRigth * self.UcloseFront) + (self.UcloseLeft * self.UcrashRigth * self.UcloseFront)

        self.Uforward = (self.UfarRigth * self.UfarLeft * self.UfarFront) + (self.UcrashLeft * self.UcrashRigth * self.UfarFront) + (self.UcrashLeft * self.UcloseRigth * self.UfarFront) + (self.UcloseLeft * self.UcrashRigth * self.UfarFront) + (self.UcloseLeft * self.UcloseRigth * self.UfarFront)

        self.Uforwardrigth = (self.UcrashLeft * self.UcloseRigth * self.UcloseFront) + (self.UcrashLeft * self.UfarRigth * self.UcloseFront) + (self.UcrashLeft * self.UfarRigth * self.UfarFront) + (self.UcloseLeft * self.UfarRigth * self.UcloseFront)+ (self.UcloseLeft * self.UfarRigth * self.UfarFront)

        self.Uturnrigth = (self.UcrashLeft * self.UfarRigth * self.UcrashFront) + (self.UcrashLeft * self.UcloseRigth * self.UcrashFront) + (self.UcrashLeft * self.UcrashRigth * self.UcrashFront) + (self.UcrashLeft * self.UcrashRigth * self.UcloseFront)+ (self.UcloseLeft * self.UcloseRigth * self.UcrashFront) + (self.UcloseLeft * self.UfarRigth * self.UcrashFront)+ (self.UfarLeft * self.UfarRigth * self.UcrashFront) + (self.UcloseLeft * self.UcloseRigth * self.UcloseFront)

    def DMC_difuzzfication (self):

        firstTrianleArea = 100.0*self.Uturnleft*(1-(self.Uturnleft/2.0))
        secondTrianleArea = 100.0*self.Uforwardleft*(1-(self.Uforwardleft/2.0))
        thirdTrianleArea = 100.0*self.Uforward*(1-(self.Uforward/2))
        fourThtrianleArea = 100.0*self.Uforwardrigth*(1-(self.Uforwardrigth/2.0))
        fithTrianleArea = 100.0*self.Uturnrigth*(1-(self.Uturnrigth/2.0))

        numerator = (-100.0*(firstTrianleArea)) + (-50.0*(secondTrianleArea)) + (0.0*(thirdTrianleArea)) + (50.0*(fourThtrianleArea)) + (100.0*(fithTrianleArea))

        deniminator = (firstTrianleArea + secondTrianleArea + thirdTrianleArea +fourThtrianleArea +fithTrianleArea)

        self.out = numerator/deniminator
        return numerator/deniminator

    def crash_evaluation(self, dist):
        
        u = (-2.5 * dist) + 1.0
        if (u <= 0): 
            return 0
        if (u >= 1): 
            return 1
        return u

    def close_evaluation(self, dist):

        if (dist == 0.4): 
            return 1
        if (dist < 0.4):
            u = 2.5 * dist
        if (dist > 0.4):
            u = (-2.5 * dist) + 2.0
        
        if (u <= 0):
            return 0

        if (u >= 1):
            return 1
        
        return u
 

    def far_evaluation(self, dist):

        if (dist < 0.8):
            u = (2.5 * dist) - 1.0

        if (dist >= 0.8):
            return 1
        if (u <= 0):
            return 0
        if (u >= 1):
            return 1

        return u

    def compute_fuzzy_out(self, left_distance, rigth_distance, front_distance):
        self.fuzzification(left_distance, rigth_distance, front_distance)
        self.Infernece_machine()
        return self.DMC_difuzzfication()
