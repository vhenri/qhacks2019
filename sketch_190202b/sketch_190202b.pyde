#from pynput.keyboard import Key, Controller
import sys
print(sys.executable)
#keyboard = Controller()

#keyboard.press('a')
#keyboard.release('a')

def setup():
    size(480, 120)
    
def draw():
    '''if mousePressed:
        fill(0)
    else:
        fill(255)
    ellipse(mouseX, mouseY, 80, 80)'''
    mouseX = 100
    mouseY = 100
    for i in range (0, 6):
        mouseX += 15
        mouseY += 15
