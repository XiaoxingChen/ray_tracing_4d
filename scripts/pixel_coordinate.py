import cv2
import matplotlib.pyplot as plt
import numpy as np

def format_coord(x, y):
        col = int(x + 0.5)
        row = int(y + 0.5)
        print("[{},{}]".format(col, row))
        return 'x=%1.4f, y=%1.4f' % (x, y)

if __name__ == "__main__":
    from sys import argv
    if len(argv) < 2:
        print("example: python pixel_coordinate.py path/to/img.png")
        quit()

    img_filename = argv[1]
    frame = cv2.imread(img_filename)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    ax = plt.gca()

    ax.format_coord = format_coord
    ax.imshow(frame)
    # plt.plot([1,2,3],[4,5,6])

    # plt.legend()
    plt.show()