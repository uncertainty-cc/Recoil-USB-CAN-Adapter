import threading

import pynput



class KeyboardController:
    def __init__(self):
        self._listener = pynput.keyboard.Listener(on_press=self.onKeyPress)
        self._is_stopped = threading.Event()
        self._is_stopped.clear()


    def onKeyPress(self, key):
        
        if type(key) == pynput.keyboard._xorg.KeyCode:
            if key.char == "1":
                self.on1Pressed()
            elif key.char == "2":
                self.on2Pressed()
            elif key.char == "3":
                self.on3Pressed()
            elif key.char == "4":
                self.on4Pressed()
            elif key.char == "5":
                self.on5Pressed()
            elif key.char == "6":
                self.on6Pressed()
            elif key.char == "7":
                self.on7Pressed()
            elif key.char == "8":
                self.on8Pressed()
            elif key.char == "9":
                self.on9Pressed()
            elif key.char == "0":
                self.on0Pressed()
            elif key.char.upper() == "A":
                self.onAPressed()
            elif key.char.upper() == "B":
                self.onBPressed()
            elif key.char.upper() == "X":
                self.onXPressed()
            elif key.char.upper() == "Y":
                self.onYPressed()
        else:
            if key == pynput.keyboard.Key.esc:
                self.onESCPressed()
            elif key == pynput.keyboard.Key.left:
                self.onLeftArrowPressed()
            elif key == pynput.keyboard.Key.right:
                self.onLeftArrowPressed()
            elif key == pynput.keyboard.Key.up:
                self.onUpArrowPressed()
            elif key == pynput.keyboard.Key.down:
                self.onDownArrowPressed()
        
        return not self._is_stopped.is_set()

    def start(self):
        self._listener.start()

    def stop(self):
        self._is_stopped.set()
        self._listener.join()
    
    def onESCPressed(self):
        pass
    def on1Pressed(self):
        pass
    def on2Pressed(self):
        pass
    def on3Pressed(self):
        pass
    def on4Pressed(self):
        pass
    def on5Pressed(self):
        pass
    def on6Pressed(self):
        pass
    def on7Pressed(self):
        pass
    def on8Pressed(self):
        pass
    def on9Pressed(self):
        pass
    def on0Pressed(self):
        pass
    def onAPressed(self):
        pass
    def onBPressed(self):
        pass
    def onXPressed(self):
        pass
    def onYPressed(self):
        pass
    def onLeftArrowPressed(self):
        pass
    def onRightArrowPressed(self):
        pass
    def onUpArrowPressed(self):
        pass
    def onDownArrowPressed(self):
        pass

if __name__ == "__main__":
    keycontroller = KeyboardController()
    keycontroller.start()

    keycontroller.onAPressed = lambda: print("A pressed")
    keycontroller.onBPressed = lambda: print("B pressed")
    keycontroller.onXPressed = lambda: print("X pressed")
    keycontroller.onYPressed = lambda: print("Y pressed")
    keycontroller.onLeftArrowPressed = lambda: print("<- pressed")
    keycontroller.onRightArrowPressed = lambda: print("-> pressed")

    try:
        while True:
            pass
    except KeyboardInterrupt:
        pass

    keycontroller.stop()
