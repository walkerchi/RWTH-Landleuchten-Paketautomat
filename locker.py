import time
import random
import numpy as np
import qrcode
import uuid
import json
import os
import cv2
from pyzbar import pyzbar as pyzbar

class PyzBarDecoder:
    def __init__(self):
        pass
    def detectAndDecode(self,img):
        barcodes = pyzbar.decode(img)
        if len(barcodes) == 0:
            return "",None,None
        barcode = barcodes[0]
        (x,y,w,h) = barcode.rect
        barcodeData = barcode.data.decode("UTF8")
        return barcodeData,[(x,y),(x+w,y),(x+w,y+h),(x,y+h)],None

class Locker:
    '''
        Constant
            H 
                height of the locker 
            N_Boden 
                number of bodens
            H_Boden 
                height of each boden 
            P_Nut 
                position of each nuts, including the bottom 
            H_Tur 
                height of the door  
            N_Tur 
                number of the door
            
    '''
    img_path = './img'
    zeit_sleep=5
    H       = 1050
    N_Boden = 6
    H_Boden = 20
    P_Nut   = [-H_Boden]+[60+70*i for i in range(12)]+[60+70*11+40+30*i for i in range(6)]
    H_Tur   = 70 
    N_Tur   = int(H//H_Tur) 
    

    def __init__(self,init_mode="normal"):
        '''
            blocks
                status of each block 
            pkgs
                status of each packages
        '''
        self.blocks = ["fobid"]+["empty"]*(len(self.P_Nut)-1)
        self.pkgs = []
        getattr(self,f"_{init_mode}_init")()
        if not os.path.exists(self.img_path):
            os.mkdir(self.img_path)
        
        self.admins = {
            'open_all':uuid.uuid1()
        }
        for  k,v in self.admins.items():
            self._gen_qr(k,{
                "operation":"admin",
                "uuid":str(v)
            })

        try:
            self._init_pi()
            self.has_pi=True
        except Exception as e:
            print("Init Pi fail")
            self.has_pi=False
    def _init_pi(self):
        import GPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        _ports = [2,]
        self.zeit_offen = 0.5
        self.IO = {
            'tur1':4,
            'tur2':17,
            'tur3':27,
            'tur4':22,
            'tur5':5,
            'tur6':6,
            'tur7':13,
            'tur8':19,
            'tur9':26,
            'tur10':18,
            'tur11':23,
            'tur12':24,
            'tur13':25,
            'tur14':12,
            'tur15':16
        }
        for v in self.IO.values():
            GPIO.setup(v,GPIO.OUT)
        
    def _normal_init(self):
        self.blocks[-self.N_Boden:] = ["boden"]*self.N_Boden
        self.stack_top = 0 
        self.heap_top  = len(self.blocks)-self.N_Boden

    def _random_init(self):
        self.blocks[-int(self.N_Boden//2):] = ["boden"]*int(self.N_Boden//2)
        self.stack_top = 0
        self.heap_top = len(self.blocks)-int(self.N_Boden//2)
        count = 0
        while count < int(self.N_Boden//2):
            r = random.randint(0,self.heap_top-1)
            if self.blocks[r] == "boden":
                continue
            else:
                self.blocks[r] = "boden"
                count+=1

    def _fixed_init(self):
        self.blocks[-int(self.N_Boden//2):] = ["boden"]*int(self.N_Boden//2)
        self.stack_top = 0
        self.heap_top = len(self.blocks)-int(self.N_Boden//2)
        self.blocks[2]="boden"
        self.blocks[5]="boden"
        self.blocks[6]="boden"

    def _push_heap(self,source):
        # print(f"push heap {source}")
        self.heap_top-=1
        assert self.blocks[self.heap_top]=="empty",f"heaptop{self.heap_top} not empty"
        self.blocks[self.heap_top]="boden"
        self.action.append((source,self.heap_top))

    def _pop_heap(self):
        # print(f"pop heap")
        assert self.heap_top<len(self.blocks),"heap above flow"
        self.blocks[self.heap_top]="empty"
        self.heap_top += 1
        return "boden"

    def _free_above(self):
        self._push_heap(self.stack_top)
        self.blocks[self.stack_top]="empty"
        for i in range(self.stack_top+1,len(self.P_Nut)):
            if self.blocks[i] == "occupy" or self.blocks[i] == "empty":
                self.blocks[i]="empty"
            else:
                break
    
    def _push(self,height):
        _nut2turbottom = dict([(i,i+1) for i in range(12)]+[(13,13),(14,14),(15,14),(16,15),(17,15),(18,15)])
        _nut2turtop = dict([(i,i) for i in range(12)]+[(13,13),(14,13),(15,14),(16,14),(17,14),(18,15)])
        
        """
            ground the height to the start end end block number
        """
        # assert all(map(lambda x:not x['status'],self.pkgs)),"all packages should be removed first"
        self.action = []
        turbottom = _nut2turbottom[self.stack_top]
        nutbottom = self.stack_top
        position = self.P_Nut[self.stack_top]+self.H_Boden+height
        for i in range(self.stack_top,self.heap_top):
            self.stack_top+=1
            # print(f"i:{i} st:{self.stack_top} nut:{self.P_Nut[i]}")
            if self.P_Nut[self.stack_top]>position:
                # self.stack_top -= 1
                break
            if self.blocks[self.stack_top] == "boden":
                self._free_above()
            assert self.blocks[self.stack_top] == "empty","stack top not empty"
            self.blocks[self.stack_top]="occupy"
        if self.blocks[self.stack_top] != "boden":
            self.action.append((self.heap_top,self.stack_top))
            self.blocks[self.stack_top] = self._pop_heap()
        turtop = _nut2turtop[self.stack_top]
        nuttop = self.stack_top
        return turtop,turbottom,nuttop,nutbottom

    def _gen_qr(self,name,data):
        qr = qrcode.QRCode(
                version =1,
                error_correction=qrcode.constants.ERROR_CORRECT_H,
                box_size=10,
                border  =4
            )
        qr.add_data(json.dumps(data))
        qr.make(fit=True)
        img = qr.make_image()
        img.save(os.path.join(self.img_path,f"{name}.jpg"))
        return qr

    def push(self,data):
        """
            Put in a package of height `height`, return the strategy placing the boden
            Parameters
            ----------
                height  
                    height of the package
            Returns
            -------
                {bd_índ:new_nut,bd_ind:new_nut}
        """

        print(f"Push {data['height']}mm")
        _blocks = self.blocks.copy()
        try:
            turtop,turbottom,nuttop,nutbottom = self._push(data['height'])
            for (u,v) in self.action.copy():
                for (u_,v_) in self.action.copy():
                    if u==v_:
                        self.action.remove((u,v))
                        self.action.remove((u_,v_))
                        self.action.append((u_,v))
                    elif v==u_:
                        self.action.remove((u,v))
                        self.action.remove((u_,v_))
                        self.action.append((u,v_))
            uid  = str(uuid.uuid1())
            self._gen_qr(f"PKG_{len(self.pkgs)}",{
                "operation":"pull",
                "uuid":uid,
                "index":len(self.pkgs)
            })
            self.pkgs.append({
                'status':True,
                'uuid':uid,
                "tur+":turtop,
                "tur-":turbottom,
                "nut+":nuttop,
                "nut-":nutbottom
            })
            print(f"Put nut {self.action[0][0]} to nut {self.action[0][1]}")
            return self.action
        except Exception as e:
            self.blocks = _blocks
            print("Push Fail")

    def _manage_tur(self,operation):
        assert len(operation) == self.N_Tur,"operation error"
        if self.has_pi:
            import RPi.GPIO as GPIO
            for i,op in enumerate(self.operation):
                if op:
                    GPIO.output(self.IO[f'tur{i+1}'],GPIO.HIGH)
            time.sleep(self.zeit_offen)
            for i,op in enumerate(self.operation):
                if op:
                    GPIO.output(self.IO[f'tur{i+1}'],GPIO.LOW)
        else:
            print("No pi")

    def _pull(self,uuid,index):
        assert index < len(self.pkgs),"index error"
        assert uuid == self.pkgs[index]['uuid'],"uuid error"
        assert self.pkgs[index]['status'] is True,"status error"
        self.pkgs[index]['status']=False
        turs = [False]*self.N_Tur
        for tur in range(self.pkgs[index]['tur-'],self.pkgs[index]['tur+']+1):
            turs[tur]=True
        for nut in range(self.pkgs[index]['nut-']+1,self.pkgs[index]['nut+']):
            self.blocks[nut]="empty"
        self._manage_tur(turs)
        
    def pull(self,data):
        print(f"Pull {data['index']}")
        _blocks = self.blocks.copy()
        try:
            self._pull(data['uuid'],data['index'])
        except Exception as e:
            print(e)
            print("Pull Fail")
            self.blocks=_blocks

    def open_all(self):
        print('open_all')
        self._manage_tur([True]*len(self.N_Tur))
        pass

    def admin(self,data):
        for k,v in self.admins.items():
            if data['uuid'] == v:
                getattr(self,k)()
        pass
    
    def __call__(self):
        debug = True
        camera = cv2.VideoCapture(0,cv2.CAP_DSHOW)
        # detector = cv2.QRCodeDetector()
        detector = PyzBarDecoder()
        while True:
            ret,frame = camera.read()
            # gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            if not hasattr(self,'exe_time') or time.time() > self.exe_time+self.zeit_sleep:
                codeinfo, points, straight_qrcode = detector.detectAndDecode(frame)
                if points is not None:
                    frame=cv2.drawContours(frame, [np.int32(points)], 0, (0, 255, 0), 4)
                    if debug:
                        print(f'{len(codeinfo)}:{codeinfo}')
                    if len(codeinfo) > 2:
                        try:
                            data = json.loads(codeinfo)
                            if 'operation' in data:
                                {'push':self.push,
                                'pull':self.pull,
                                'admin':self.admin}[data['operation']](data)
                            self.exe_time = time.time()
                            print(self)
                        except:
                            raise Exception(f'Erro Info:{codeinfo}')
            cv2.imshow("camera",frame)
            if cv2.waitKey(1) == ord('q'):
                break


    def __repr__(self):
        for i in range(len(self.blocks)):
            print(f"--{i:^4}--",end="")
        print("")
        for b in self.blocks:
            print(f"| {b:^6}",end="")
        print("|")
        print("--------"*len(self.blocks))
        return ""

           
if __name__ == '__main__':
    # l = Locker(init_mode="fixed")
    # print(l)
    # print(l.push({'height':300}))
    # print(l.push({'height':300}))
    # print(l.pull({}))
    Locker(init_mode='normal')()