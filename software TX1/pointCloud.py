import numpy as np
import cv2
import matplotlib.pyplot as plt
from functions import *

def main(imgLeft,imgRigth):
	
	######### PATH ALLE IMMGINI DELLA SCACCHIERA PER CALIBRAZIONE #########
	#path_left = 'Immagini/Calibrate/left_path/'
	#path_rigth = 'Immagini/Calibrate/right_path/'

	#imgLeft = cv2.imread('Immagini/DaMettereTx1/Test/Left24.jpg',0)
	#imgRigth = cv2.imread('Immagini/DaMettereTx1/Test/Right24.jpg',0)

	imgLeft = np.uint8(imgLeft)
	imgRigth = np.uint8(imgRigth)

	im = imgLeft # per proiezione 3d

	imgLeft = cv2.cvtColor(imgLeft, cv2.COLOR_BGR2GRAY)
	imgRigth = cv2.cvtColor(imgRigth, cv2.COLOR_BGR2GRAY)

	plt.imshow(imgLeft,'gray')
    	plt.show()

	plt.imshow(imgRigth,'gray')
    	plt.show()
	
	image_size = imgLeft.shape[::-1]
	
	'''	
	numero_im = 28

	### FIND INTRINSIC PARAMETER ###
	print 'findIntrinsicParameter'
	mtxL, distL, mtxR, distR, obj_points, img_left_points, img_right_points = calibrareCamere(path_left,path_rigth,numero_im)

	### UNDISTORT IMAGE ### non sempre funziona, roi=(0,0,0,0), e la rettifica e peggiore
	#imgLeft, imgRigth = undistortImage(imgLeft,imgRigth,mtxL,distL,mtxR,distR)

	### STEREO CALIBRATION ###
	print 'stereoCalibrate'
	stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
	stereocalib_retval, mtxL, distL, mtxR, distR, R, T, E, F = cv2.stereoCalibrate(obj_points,img_left_points,img_right_points,
		                                                                      mtxL,distL,mtxR,distR,image_size,
		                                                                      flags = cv2.CALIB_FIX_INTRINSIC,
		                                                                      criteria = stereocalib_criteria)
	
	save(mtxL,"mtxL")
	save(distL,"distL")
	save(mtxR,"mtxR")
	save(distR,"distR")
	save(R,"R")
	save(T,"T")
	'''
	#######################################################################################################################################################################
	################################### PARTE DA QUI, LA PRIMA PARTE SI ESEGUE SOLO SE SI CAMBIA POSIZIONE DELLE CAMERE ###################################################
	#######################################################################################################################################################################	
	mtxL = np.asarray(read("mtxL"))
	distL = np.asarray(read("distL"))
	mtxR = np.asarray(read("mtxR"))
	distR = np.asarray(read("distR"))
	R = np.asarray(read("R"))
	T = np.asarray(read("T"))


	### STEREO RECTIFICATION ###
	print 'rettificaImmagini'
	imgLeftRemap, imgRightRemap, Q = rettificaImmagini(imgLeft,imgRigth,mtxL, distL, mtxR, distR, image_size,

		                                      R, T)
	#plt.imshow(imgLeftRemap,'gray')
	#plt.show()
	#plt.imshow(imgRightRemap,'gray')
	#plt.show()

	### STEREO CORRESPONDENCE ###
	print 'Stereo Correspondence'
	disp = stereoMatch(imgLeftRemap,imgRightRemap);

	'''
	### SUBPIXEL REFINEMENT ###
	print 'Disaprity Refinement'
	disparity = Refinement(disparitySGBM)
	plt.imshow(disparity,'gray')
	plt.show()
	'''
	### POINT CLOUD ###
	print 'computePointCloud'
	pointCloud(disp, im, imgRigth, mtxL, distL, mtxR, distR, image_size, R, T, Q)

	return disp
