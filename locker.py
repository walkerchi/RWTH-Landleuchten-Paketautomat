import time
import random
import numpy as np
import qrcode
import uuid
import json
import os
import cv2
from pyzbar import pyzbar as pyzbar

# import pyzbar


class PyzBarDecoder:
    """
    Decoder für QR-Code
    """

    def detectAndDecode(self, img):
        """
        Code spüren und decodieren
        """
        barcodes = pyzbar.decode(img)
        if len(barcodes) == 0:
            return "", None, None
        barcode = barcodes[0]
        (x, y, w, h) = barcode.rect
        barcodeData = barcode.data.decode("UTF8")
        return barcodeData, [(x, y), (x + w, y), (x + w, y + h), (x, y + h)], None


class Locker:
    """
    Constant
        H
            height of the locker
        N_Boden
            number of bodens
        H_Boden
            height of each boden
        P_Nut
            Position of each nuts, including the bottom
        N_Tur
            number of the door
        Nut2TurBottom
            nut as the bottom the lowest tur should open
            nut begin with 0(the bottom of locker not the real nut)
            tur begin with 1
        Nut2TurTop
            nut as the top the highest tur should open

    """

    img_path = "./img"  # Speicherordner für QR-Codes
    zeit_sleep = 5
    H = 1050
    N_Boden = 6
    H_Boden = 20
    P_Nut = (
        [-H_Boden]
        + [60 + 70 * i for i in range(12)]
        + [60 + 70 * 11 + 40 + 30 * i for i in range(7)]
    )
    N_Tur = 15
    Nut2TurBottom = dict(
        [(i, i + 1) for i in range(13)]
        + [(13, 13), (14, 14), (15, 14), (16, 15), (17, 15), (18, 15), (19, 15)]
    )
    Nut2TurTop = dict(
        [(i, i) for i in range(13)]
        + [(13, 13), (14, 13), (15, 14), (16, 14), (17, 14), (18, 15), (19, 15)]
    )

    def __init__(self, init_mode="normal"):
        """
        Initialisierung von Modus

        blocks
            status of each block /Position
        pkgs
            status of each package
        """
        self.blocks = (
            ["fobid"] + ["empty"] * (len(self.P_Nut) - 2) + ["fobid"]
        )  # Alle vorhandenen Positionen
        self.pkgs = []  # Die Belegungsstatus der Paketen
        getattr(self, f"_{init_mode}_init")()  # Get Attribute
        if not os.path.exists(self.img_path):
            os.mkdir(self.img_path)  # ein Ordner für QR-Codes wird generiert

        self.admins = {
            "open_all": str(uuid.uuid1())
        }  # ein zufälliges Passwort für Open-All wird generiert
        for k, v in self.admins.items():
            self._gen_qr(
                k, {"operation": "admin", "uuid": v}
            )  # QR-Codes werden für jedes UUID-Passwort generiert

        try:
            self._init_pi()
            self.has_pi = True  # Wenn Raspi angeschlossen ist, wird _init_pi() erfolgreich durchgeführt
        except Exception as e:
            print(
                "Init Pi fail"
            )  # Wenn Raspi nicht angeschlossen ist, wird Error ausgedruckt.
            self.has_pi = False

    def _init_pi(self):
        """
        Initialisierung für Raspi
        """
        import RPi.GPIO as GPIO  # GPIO Pins

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.zeit_offen = 0.5  # Strom wird für 0,5s angeschlossen
        self.IO = {
            "tur1": 4,
            "tur2": 17,
            "tur3": 27,
            "tur4": 22,
            "tur5": 5,
            "tur6": 6,
            "tur7": 13,
            "tur8": 19,
            "tur9": 26,
            "tur10": 18,
            "tur11": 23,
            "tur12": 24,
            "tur13": 25,
            "tur14": 12,
            "tur15": 16,
        }  # GPIO-Pins werden mit Schlössen zugeteilt
        for v in self.IO.values():
            GPIO.setup(v, GPIO.OUT)  # Jedes Pin wird als OutPut-Mode eingestellt

    def _normal_init(self):
        """
        Normal-Initmode: Am Anfang werden alle Platten zur einen Ende verschoben.
        """
        self.blocks[-self.N_Boden - 1 : -1] = ["boden"] * self.N_Boden
        self.stack_top = 0  # "Stapel": Die Belegung von Paketen
        self.heap_top = (
            len(self.blocks) - self.N_Boden - 1
        )  # "Haufen": Belegung von Platten

    def _random_init(self):
        """
        Ein paar Platten sind im Haufen, die anderen befinden sich in zufälligen Positionen
        """
        self.blocks[-int(self.N_Boden // 2) - 1 : -1] = ["boden"] * int(
            self.N_Boden // 2
        )
        self.stack_top = 0
        self.heap_top = len(self.blocks) - int(self.N_Boden // 2) - 1
        count = 0
        while count < int(self.N_Boden // 2):
            r = random.randint(1, self.heap_top - 1)
            if self.blocks[r] == "boden":
                continue
            else:
                self.blocks[r] = "boden"
                count += 1

    def _fixed_init(self):
        """
        Nur zum Testen!!!
        """
        self.blocks[-int(self.N_Boden // 2) - 1 : -1] = ["boden"] * int(
            self.N_Boden // 2
        )
        self.stack_top = 0
        self.heap_top = len(self.blocks) - int(self.N_Boden // 2) - 1
        self.blocks[2] = "boden"
        self.blocks[5] = "boden"
        self.blocks[6] = "boden"

    def _push_heap(self, source):
        """
        push: Platten werden in die Stapel eingesteckt
        """
        self.heap_top -= 1
        assert (
            self.blocks[self.heap_top] == "empty"
        ), f"heaptop{self.heap_top} not empty"
        self.blocks[self.heap_top] = "boden"
        self.action.append(
            (source, self.heap_top)
        )  # Die Bewegungsvorgang wird verzeichnet für künftige Anweisung zu Mitarbeiter

    def _pop_heap(self):
        """
        Platten werden aus dem Haufen entfernt
        """
        assert self.heap_top < len(self.blocks) - 1, "heap above flow"
        self.blocks[self.heap_top] = "empty"
        self.heap_top += 1
        return "boden"

    def _is_heap_empty(self):
        return self.heap_top == len(self.blocks) - 1

    def _push(self, height):
        _nut2turbottom = self.Nut2TurBottom
        _nut2turtop = self.Nut2TurTop

        """
        Ein Paket wird ins Fach eingelegt
        """
        self.action = []
        turbottom = _nut2turbottom[
            self.stack_top
        ]  # Das niedrigste Türsegment des Faches
        nutbottom = self.stack_top  # Die untere Plattenposition des Faches
        package_top = (
            self.P_Nut[self.stack_top] + self.H_Boden + height
        )  # Die Obere Höhe des Paketes
        for i in range(
            self.stack_top, self.heap_top
        ):  # Schleife durchführen, bis die Postion der zweiten Platte gerade das Paket einhält
            self.stack_top += 1
            # print(f"i:{i} st:{self.stack_top} nut:{self.P_Nut[i]}")
            if self.P_Nut[self.stack_top] >= package_top:
                # self.stack_top -= 1
                break
            if self.blocks[self.stack_top] == "boden":
                self._push_heap(self.stack_top)
                self.blocks[self.stack_top] = "empty"
            assert self.blocks[self.stack_top] == "empty", "stack top not empty"
            self.blocks[self.stack_top] = "occupy"
        if (  # 如果堆为空, 向上找板子, 如果不为空, 从堆里找板子
            self.blocks[self.stack_top] != "boden"
            and self.blocks[self.stack_top] != "fobid"
        ):
            if self._is_heap_empty():
                flag = True
                for i in range(self.stack_top, self.heap_top):
                    if self.blocks[i] == "boden":
                        self.blocks[i] = "empty"
                        self.blocks[self.stack_top] = "boden"
                        self.action.append((i, self.stack_top))
                        flag = False
                        break
                if flag:
                    for i in range(self.stack_top, self.heap_top):
                        self.blocks[i] = "occupy"
            else:
                self.action.append((self.heap_top, self.stack_top))
                self.blocks[self.stack_top] = self._pop_heap()
        turtop = _nut2turtop[self.stack_top]  # das oberste Türsegment
        nuttop = self.stack_top  # die oberste Plattenpositon
        return turtop, turbottom, nuttop, nutbottom

    def _gen_qr(self, name, data):
        """
        QR-Code wird generiert
        """
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=10,
            border=4,
        )
        qr.add_data(json.dumps(data))  # dict wird zum str umgewandelt
        qr.make(fit=True)
        img = qr.make_image()
        img.save(
            os.path.join(self.img_path, f"{name}.jpg")
        )  # QR-Code wird im vorher generierten Ordner gespeichert
        return qr

    def _action_hook(self, action):
        """
        Wird fürs Einpassen der schlitten.py als Schnittstelle verwendet, siehe schlitten.py
        """
        return action

    def push(self, data):
        """
        Put in a package of data [`height`], return the strategy placing the boden
        Parameters
        ----------
            data
                data must have a key of 'height'
        Returns
        -------
            {bd_índ:new_nut,bd_ind:new_nut}


        Der Kurier gibt das Maß des Pakets ein, das Raspi gibt die Anweisung für Verschiebung der Platte
        """

        print(f"Push {data['height']}mm")  # Das Paket mit Höhe 'height' wird eingelegt
        _blocks = self.blocks.copy()
        try:
            turtop, turbottom, nuttop, nutbottom = self._push(data["height"])
            self.action = self._action_hook(self.action)
            for (u, v) in self.action.copy():
                for (u_, v_) in self.action.copy():
                    if u == v_:
                        self.action.remove((u, v))
                        self.action.remove((u_, v_))
                        self.action.append((u_, v))
                    elif v == u_:
                        self.action.remove((u, v))
                        self.action.remove((u_, v_))
                        self.action.append((u, v_))
            uid = str(uuid.uuid1())
            self._gen_qr(
                f"PKG_{len(self.pkgs)}",
                {"operation": "pull", "uuid": uid, "index": len(self.pkgs)},
            )
            self.pkgs.append(
                {
                    "status": True,
                    "uuid": uid,
                    "tur+": turtop,
                    "tur-": turbottom,
                    "nut+": nuttop,
                    "nut-": nutbottom,
                }
            )  # Informationen des eingelegten Paketes und des zugehörigen Faches
            if len(self.action) > 0:
                print(
                    f"Put nut {self.action[0][0]} to nut {self.action[0][1]}"
                )  # Baustelle
            else:
                print("No action")  # Baustelle
            return self.action
        except Exception as e:
            self.blocks = _blocks
            print(e)
            print("Push Fail")  # Baustelle

    def _print_tur(self, operation):
        # für terminal output
        for i in range(len(operation)):
            print(f"--{i+1:^3}--", end="")
        print("")
        for i in operation:
            tmp = {True: "open", False: "close"}
            print(f"|{tmp[i]:^6}", end="")
        print("|")
        for i in range(len(operation)):
            print(f"-------", end="")
        print("")

    def _print_nut(self):
        # für terminal output

        for i in range(len(self.blocks)):
            print(f"--{i:^4}--", end="")
        print("")
        for b in self.blocks:
            print(f"| {b:^6}", end="")
        print("|")
        print("--------" * len(self.blocks))
        return ""

    def _manage_tur(self, operation):
        """
        Parameters
        ----------
        operation
            operation of each tur, if True open the tur.
            list of boolean [False,False,....,True,True,...], with length [N_Tur]
        """
        assert len(operation) == self.N_Tur, "operation error"
        if self.has_pi:
            import RPi.GPIO as GPIO

            for i, op in enumerate(operation):
                if op:
                    GPIO.output(self.IO[f"tur{i+1}"], GPIO.HIGH)
            time.sleep(self.zeit_offen)
            for i, op in enumerate(operation):
                if op:
                    GPIO.output(self.IO[f"tur{i+1}"], GPIO.LOW)
        else:
            print("No pi")
        self._print_tur(operation)

    def _pull(self, uuid, index):
        """
        Türen werden geöffnet
        """
        assert index < len(self.pkgs), "index error"  # 包裹序号不在范围内
        assert uuid == self.pkgs[index]["uuid"], "uuid error"  # uuid匹配
        assert self.pkgs[index]["status"] is True, "status error"  # 包裹的状态为True
        self.pkgs[index]["status"] = False  # 该包裹准备被取出, 状态变为False
        turs = [False] * self.N_Tur  # 门初始状态为False, 即关闭
        for tur in range(self.pkgs[index]["tur-"], self.pkgs[index]["tur+"] + 1):
            turs[tur - 1] = True  # 该储物空间的门全部打开
        for nut in range(self.pkgs[index]["nut-"] + 1, self.pkgs[index]["nut+"]):
            self.blocks[nut] = "empty"  # 所有的position从occupy变为empty
        self._manage_tur(turs)  # 执行_manage_tur()

    def pull(self, data):
        print(f"Pull {data['index']}")
        _blocks = self.blocks.copy()
        try:
            self._pull(data["uuid"], data["index"])
        except Exception as e:
            print(e)
            print("Pull Fail")
            self.blocks = _blocks

    def open_all(self):
        print("open_all")
        self._manage_tur([True] * self.N_Tur)
        for i, block in enumerate(self.blocks):
            if block == "occupy":
                self.blocks[i] = "empty"
        self._normal_init()

    def admin(self, data):
        for k, v in self.admins.items():
            if data["uuid"] == v:
                getattr(self, k)()
        pass

    def parse_qr(self, img):
        if not hasattr(self, "qr_detector"):
            self.qr_detector = PyzBarDecoder()
        codeinfo, points, _ = self.qr_detector.detectAndDecode(img)
        if points is not None:
            img = cv2.drawContours(img, [np.int32(points)], 0, (0, 255, 0), 4)
        # print(f'{len(codeinfo)}:{codeinfo}')
        status = False
        try:
            data = json.loads(codeinfo)
            if "operation" in data:
                {"push": self.push, "pull": self.pull, "admin": self.admin}[
                    data["operation"]
                ](data)
            status = True
        except Exception as e:
            # print(e)
            # print(f'Erro Info:{codeinfo}')
            pass
        return img, status

    def __call__(self):
        # 输入空括号就会执行这个函数
        debug = True
        camera = cv2.VideoCapture(0)
        # detector = cv2.QRCodeDetector()
        while True:
            ret, frame = camera.read()
            if (
                not hasattr(self, "exe_time")
                or time.time() > self.exe_time + self.zeit_sleep
            ):
                # gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                frame, status = self.parse_qr(frame)
                if status:
                    self.exe_time = time.time()
                    print(self)
            cv2.imshow("camera", frame)
            if cv2.waitKey(1) == ord("q"):
                break

    def __repr__(self):
        return self._print_nut()


if __name__ == "__main__":  # 在当前文件下会执行, 在其他文件下不会运行

    l = Locker(init_mode="random")
    print(l)
    # l()

    l.push({"height": 200})
    print(l)
    l.push({"height": 40})
    print(l)
    l.push({"height": 70})
    print(l)
    l.push({"height": 100})
    print(l)
    l.push({"height": 200})
    print(l)
    l.push({"height": 40})
    print(l)
    l.parse_qr(cv2.imread("img/PKG_0.jpg"))
    l.parse_qr(cv2.imread("img/PKG_1.jpg"))
    l.parse_qr(cv2.imread("img/PKG_2.jpg"))
    l.parse_qr(cv2.imread("img/PKG_3.jpg"))
    l.parse_qr(cv2.imread("img/PKG_4.jpg"))
    l.parse_qr(cv2.imread("img/open_all.jpg"))
    print(l)

    # Locker(init_mode='random')()
