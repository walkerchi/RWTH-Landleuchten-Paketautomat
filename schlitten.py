from locker import Locker

'''
+*6----- [1050mm]  ------0*+
|                          | 
|<4> 195mm              <1>|    
|                          |
+-------           --------+
|*5 20mm [835mm]         1*|
+-------           --------+
|                          |
|<3> 195mm              <2>|
|                          |
+-------           --------+
|*4 20mm [620mm]         2*|
+-------           --------+
|                          |
|                          |
|<2> 300mm              <3>|
|                          |
|                          |
+-------           --------+
|*3 20mm [300mm]         3*|
+-------           --------+
|                          |
|*2 [200m]               4*|
|<1> 300mm              <4>|
|*1 [100m]               5*|
|                          |
+*0-----           ------6*+
'''

class Schlitten(Locker):
    H       = 1050
    N_Boden = 2
    H_Boden = 4
    P_Nut   = [-H_Boden,100,200,300,620,835,1050]
    N_Tur   = 4
    Nut2TurBottom = {0:1,1:2,2:3,3:4,4:4,5:4}
    Nut2TurTop    = {1:1,2:2,3:3,4:4,5:4,6:4}
    def __new__(cls,*arg, **kwarg):
        cls.P_Nut = list(map(lambda x:cls.H-x-cls.H_Boden,cls.P_Nut))
        cls.P_Nut.reverse()
        return super().__new__(cls)

    def _init_pi(self):
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.zeit_offen = 0.5
        self.IO = {
            'tur1':5,
            'tur2':6,
            'tur3':13,
            'tur4':19,
        }
        for v in self.IO.values():
            GPIO.setup(v,GPIO.OUT)

    def _action_hook(self, action):
        self.action = map(lambda x:(len(self.P_Nut)-1-x[0],len(self.P_Nut)-1-x[1]),action)
        return list(self.action)

    def _print_tur(self,operation):
        for i in range(len(operation)):
            print(f"--{i+1:^3}--",end="")
        print("")
        for i in operation[::-1]:
            tmp = {
                True:"open",
                False:"close"
            }
            print(f"|{tmp[i]:^6}",end="")
        print("|")
        for i in range(len(operation)):
            print(f"-------",end="")
        print("")

    def _print_nut(self):
        for i in range(len(self.blocks)):
            print(f"--{i:^4}--",end="")
        print("")
        for b in self.blocks[::-1]:
            print(f"| {b:^6}",end="")
        print("|")
        print("--------"*len(self.blocks))
        return ""
    

if __name__ == '__main__':
    import cv2
    # l = Test1(init_mode="normal")
    # print(l)
    # l.push({'height':300})
    # print(l)
    # l.push({'height':400})
    # print(l)
    # l.parse_qr(cv2.imread('img/PKG_0.jpg'))
    # l.parse_qr(cv2.imread('img/PKG_1.jpg'))
    # l.parse_qr(cv2.imread('img/open_all.jpg'))
    # print(l)
    l = Schlitten(init_mode="normal")
    print(l)
    l.push({'height':300})
    print(l)
    l.push({'height':300})
    print(l)
    l.parse_qr(cv2.imread('img/PKG_0.jpg'))
    l.parse_qr(cv2.imread('img/PKG_1.jpg'))
    l.parse_qr(cv2.imread('img/open_all.jpg'))
    print(l)
