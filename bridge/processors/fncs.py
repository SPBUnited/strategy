import time

class PrintTime:
    def __init__(self, sleep):
        self.oldTime = -1
        self.vivod = 0
        self.sleep = sleep

    def myPrint(self, text = "", sep = " ", end = "\n"):
        if self.vivod == 1:
            try:
                print(text, sep=sep, end=end)
            except:
                print("err print")

    def updateTime(self):
        if self.oldTime != time.time() * 1000 // self.sleep:
            self.vivod = 1
            self.oldTime = time.time() * 1000 // self.sleep
        else:
            self.vivod = 0