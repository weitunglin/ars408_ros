import matplotlib.pyplot as plt
import cv2
from matplotlib.backend_bases import MouseButton


filename = '/home/allen/micromax/catkin_ws/src/ARS408_ros/ars408_ros/1201_17d_1.png'

img = cv2.imread(filename)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

fig, ax = plt.subplots()
im = ax.imshow(img)

checkpoints = []

def on_click(event):
    if event.button is MouseButton.LEFT:
        ckpt = [event.x, event.y]
        checkpoints.append(ckpt)
        print(ckpt)
        print(checkpoints)


plt.connect('button_press_event', on_click)

plt.show()
