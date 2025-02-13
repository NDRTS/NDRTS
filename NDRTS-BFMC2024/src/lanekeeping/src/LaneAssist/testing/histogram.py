import matplotlib.pyplot as plt
import cv2


path = 'path to an image'
cap = cv2.VideoCapture("video_repository/real_roads_drive/*IMG_2983.mp4")
number_of_frame = 100
_, src = cap.read()
for i in range(number_of_frame):
    _, src = cap.read()
    
frame = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
bottom_offset = 20
bottom_perc = 0.58
slices = 30
colours = [(255, 0, 0),(0, 255, 0),(0,0,255),(0, 0, 0),(255, 0, 255),(255, 255, 0),(128, 0, 255),(255, 128, 0),(128, 0, 128),(0, 128, 128),(128, 128, 0)]
height, width = frame.shape[0], frame.shape[1]
bottom_row_index = height - bottom_offset
end = int((1 - bottom_perc) * height)
step = int(-(height * bottom_perc / slices))
real_slices = int((end - bottom_row_index) // step)
top_row_index = bottom_row_index + real_slices * step
x = [int(i) for i in range(width)]
cnt = 0 

for height in range(bottom_row_index, top_row_index - 1, step):
    i = (height - bottom_row_index)// step % len(colours)
    src = cv2.line(src, (0,height), (width,height), colours[i])
    histogram = [int(x) for x in frame[height]]
    plt.plot(x, histogram, color=(colours[i][2]/255, colours[i][1]/255, colours[i][0]/255))
    cv2.imshow("src", src)
    cv2.waitKey(10)
    plt.xlabel('Width')
    plt.ylabel('Height')
    plt.title('Histogramm of the image slice')
    plt.show()
    plt.clf()
