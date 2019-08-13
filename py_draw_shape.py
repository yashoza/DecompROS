from PIL import Image, ImageFont, ImageDraw
import colorsys
import numpy as np
from io import StringIO

# im = Image.new('RGB', (100,200), (255,0,0))
im = Image.new('1', (200,200))

dr = ImageDraw.Draw(im)

dr.rectangle(((1,1),(199,199)), fill="black", outline = "white")
dr.ellipse((150,150,190,190), fill="white", outline = "white")
# dr.ellipse((80,150,30,190), fill="white", outline = "white")
dr.rectangle(((150,30),(100,70)), fill="white", outline = "white")
dr.polygon(((10,170),(170,90),(65,65)), fill="white", outline = "white")
# dr.rectangle(((10,10),(65,65)), fill="white", outline = "white")

im.show()
print im.size

image = np.array(im)
print image.size

kernel1 = np.array([[1,1,1],[1,1,1],[1,1,1],[1,1,1],[1,1,1]])
kernel2 = np.array([[1,1,1,1,1],[1,1,1,1,1],[1,1,1,1,1]])

# output1 = (kernel1*image[10:15,10:13]).sum()
# output2 = (kernel2*image[10:13,10:15]).sum()

# write the kernel to the file
# file = open("the_matrix_circle.txt", "w")
# for i in range(0, 19):
# 	for j in range(0, 19):
# 		if image[i,j] == True:
# 			file.write("1 ")
# 		else:
# 			file.write("0 ")

# 	file.write("\n")

# this is for loading the data as np array
data = np.loadtxt("the_matrix_circle.txt")
# print data

# for checking the conolution
# for i in range(0, 19):
# 	for j in range(0, 19):
# 		if image[i,j] == data[i,j]:
# 			print "1 "
# 		else:
# 			print "0 "

print("shape_simple_5.png")

im.save("shape_simple_5.png")

