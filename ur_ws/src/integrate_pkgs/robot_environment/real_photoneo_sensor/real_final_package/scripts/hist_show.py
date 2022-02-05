import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

gain = 0.025
img = cv2.imread("/home/ericlab/tsuchida/real_image/image_1_1_16_45_3.jpg")
# img = cv2.imread("/home/ericlab/tsuchida/ros_package/integ_ws/src/tsuchida.jpg")

hist = np.zeros(256)
start = time.time()
count = 0
sum = 0
waru = 10
for i in range((int)(img.shape[0] / waru)):
    for j in range((int)(img.shape[1] / waru)):
        # hist[img[waru*i][waru*j]] += 1
        count = count + 1
        sum = sum + img[waru*i][waru*j][0]

# histimg = np.zeros((1200, 256, 3), np.uint8)
# for i in range(256):
#     histimg = cv2.line(histimg, (i, 0), (i, int(hist[i]*gain)), (200, 255, 255), 1)

# plt.axis('off')
# fig, preplt = plt.subplots()
# preplt.imshow(cv2.cvtColor(histimg, cv2.COLOR_BGR2RGB))
# preplt.invert_yaxis()
# plt.show()
# cv2.imwrite("hist_orig.jpg", histimg)
end = time.time()
print("average: ", sum/count)

print("time: ", end - start)
print("count: ", count)
print(img[4][23])