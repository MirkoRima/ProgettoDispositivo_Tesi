import numpy as np
import cv2
from scipy import signal


def edge(img):
	x,y,z = img.shape
	b,g,r = cv2.split(img)
	
	#MASK
	mask = np.ones((x,y))
	for i in range(1,x-1):
		for j in range(1,y-1):
			if r[i,j] > 250 or g[i,j] > 250 or b[i,j] > 250:
				mask[i-1,j-1] = 0
				mask[i,j-1] = 0
				mask[i+1,j-1] = 0
				mask[i-1,j] = 0
				mask[i,j] = 0
				mask[i+1,j] = 0
				mask[i-1,j+1] = 0
				mask[i,j+1] = 0
				mask[i+1,j+1] = 0

	'''	
	#GRAY WORLD CORRECTION
	meanR = np.mean(r)
	meanG = np.mean(g)
	meanB = np.mean(b)	
	
	correction = np.divide(np.power(np.min([meanR,meanG,meanB]),2),np.power(np.max([meanR,meanG,meanB]),2))
	m = np.divide(np.add(np.add(meanR,meanG),meanB),3)
	corrR = pow(m/meanR,correction)
	corrG = pow(m/meanG,correction)
	corrB = pow(m/meanB,correction)
	'''
	# GRAY EDGE
	hk = [[-1,0,1]]
	vk = [[-1],[0],[1]]
	
	gr = np.sqrt(np.add(np.power(signal.convolve(r, hk,'same','direct'),2),np.power(signal.convolve(r, vk,'same'),2)))
	#meanedgeR = np.mean(np.multiply(gr,mask))
	gg = np.sqrt(np.add(np.power(signal.convolve(g, hk,'same','direct'),2),np.power(signal.convolve(g, vk,'same'),2)))	
	#meanedgeG = np.mean(np.multiply(gg,mask))
	gb = np.sqrt(np.add(np.power(signal.convolve(b, hk,'same','direct'),2),np.power(signal.convolve(b, vk,'same'),2)))	
	#meanedgeB = np.mean(np.multiply(gb,mask))
	

	mnorm = 6.0;
	gr = np.float32(gr)
	gg = np.float32(gg)
	gb = np.float32(gb)
	gr = np.power(gr,mnorm)
	gg = np.power(gg,mnorm)
	gb = np.power(gb,mnorm)	

	whiter = np.power(np.multiply(gr,mask).sum(),1/mnorm) 
	whiteg = np.power(np.multiply(gg,mask).sum(),1/mnorm)
	whiteb = np.power(np.multiply(gb,mask).sum(),1/mnorm)	

	mean = np.sqrt(np.divide(np.add(np.add(np.power(whiter,2),np.power(whiteg,2)),np.power(whiteb,2)),3))
	#print mean	

	finalr = np.divide(r, whiter/mean)
	finalg = np.divide(g, whiteg/mean)
	finalb = np.divide(b, whiteb/mean)
	'''
	M = np.max(finalg)
	finalr = np.multiply(np.divide(finalr, M), 255)
	finalg = np.multiply(np.divide(finalg, M), 255)
	finalb = np.multiply(np.divide(finalb, M), 255)
	'''
	out = np.dstack((finalb,finalg,finalr))

	return out
