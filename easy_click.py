from pynput import mouse
from pynput.keyboard import Controller as KC
from pynput.mouse import Controller as MC
from pynput.mouse import Button
import time
k = KC()
m = MC()
i = 1

def on_click(x, y, button, pressed):
    global i
    if button == mouse.Button.right:
        if  i:
            k.press('e')
            k.release('e')
        else:
            pass
        i = 1-i
        
    
# Collect events until released
with mouse.Listener(on_click=on_click) as l:
    l.join()