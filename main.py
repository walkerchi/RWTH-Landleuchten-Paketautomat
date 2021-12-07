
from numpy.lib.arraysetops import isin
import qrcode 
import json
from PIL import Image


class QrCode:
    __all__=['_qr','_data']
    def __init__(self):
        pass

    def config(self,
                data = None,
                version =2,
                box_size=5,
                border  =4
                ):
        self._qr = qrcode.QRCode(
            version =version,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=box_size,
            border  =border
        )

        if data is not None:
            data = json.dumps(data)
            self._qr.add_data(data)
            self._qr.make(fit=True)
            self._data = data
        
        return self
        
    def save_qr(self,path='tmp.png'):
        import matplotlib.pyplot as plt
        assert hasattr(self,'_qr') ,"QR code not config or detect"
        img = self._qr.make_image()
        img.save(path)
        return self

    def save_data(self,path='tmp.json'):
        assert hasattr(self,'_data')
        json.dump(self._data,open(self._data,'w'))
        return self

    def save(self,path='tmp'):
        self.save_qr(path+'.png')
        self.save_data(path+'.json')
        return self

    def detect(self,debug=False):
        import cv2
        import numpy as np
        camera = cv2.VideoCapture(0)
        detector = cv2.QRCodeDetector()
        while True:
            ret,frame = camera.read()
            # gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            codeinfo, points, straight_qrcode = detector.detectAndDecode(frame)
            if points is not None:
                frame=cv2.drawContours(frame, [np.int32(points)], 0, (0, 255, 0), 4)
                if debug:
                    print(codeinfo)
                if not debug and len(codeinfo) > 2:
                    try:
                        data = json.loads(codeinfo)
                    except:
                        raise Exception(f'Erro Info:{codeinfo}')
                    return data
            cv2.imshow("camera",frame)
            if cv2.waitKey(1) == ord('q'):
                break
            

if __name__ == '__main__':
    # QrCode().config(data={"width":10,"height":20}).save()
    print(QrCode().detect())

