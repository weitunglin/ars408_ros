import matplotlib.pyplot as plt
import cv2
import sys
from matplotlib.backend_bases import MouseButton


filename = sys.argv[1]

img = cv2.imread(filename)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

fig, ax = plt.subplots()
im = ax.imshow(img)

checkpoints = []

def on_click(event):
    if event.button is MouseButton.LEFT:
        ckpt = [event.xdata, event.ydata]
        checkpoints.append(ckpt)
        print(ckpt)
        print(checkpoints)


plt.connect('button_press_event', on_click)

plt.show()
