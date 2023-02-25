## Imports
import cv2
import colorsys

## Get Contour Average Color Around Center
def getColor(img, contour, rect, sampleSize=10):
	rgb = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB) # this may actually BGR?
	size = rgb.shape

	center = [int(rect[0] + (rect[2]/2)), int(rect[1] + (rect[3]/2))]
	ssize = int(sampleSize/2)

	avgC1 = []
	avgC2 = []
	avgC3 = []

	for x in range(center[0] - ssize, center[0] + ssize + 1):
		if x >= size[1]:
			continue

		for y in range(center[1] - ssize, center[1] + ssize + 1):
			if y < size[0] and cv2.pointPolygonTest(contour, (x, y), False):
				avgC1.append(rgb[y][x][0])
				avgC2.append(rgb[y][x][1])
				avgC3.append(rgb[y][x][2])

	if len(avgC1) == 0:
		return -1

	return [sum(avgC3)/len(avgC3), sum(avgC2)/len(avgC2), sum(avgC1)/len(avgC1)]

## RGB To HSV
def getHSV(r, g, b):
	hsv = colorsys.rgb_to_hsv(r, g, b)
	return [hsv[0] * 360, hsv[1], hsv[2]/255]

## Distance Between HSVs
def distHSV(hsv1, hsv2):
	return (abs(hsv1[0] - hsv2[0]) + abs(hsv1[2] - hsv2[2]))/2
