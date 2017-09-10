import serial

from E import edge
from pointCloud import main
import numpy as np
import cv2
import time

ard = serial.Serial('/dev/ttyACM0', 9600, timeout= None) #controllare se 0 o x
#Nella variabile ard leggiamo i dati provenienti da Arduino
# se errore controllare che il device sia '/dev/ttyACM0' o '/dev/ttyACM1' e cambiarlo nel codice sopra del serial.Serial()

nc = 2 #numero camere che si utilizzano 
camera = [0]*nc
frame = [0]*nc
oggetto = False

while True:
	x = ard.readline()
	print (x)
	#nella variabile x inseriamo i valori della lettura e li mandiamo in stampa
	if "ON" in x and not(oggetto):
	
	 #STOP ARDUINO
	 	ard.write('S') # S = STOP ARDUINO
	
		Oldframe = [0]*nc
		control = [0]*nc  	 #array di boolean true se frame[i]==frame[i-1]
		flag = True  		 #false se scatto	
		while flag:
			for i in range(0,nc):  
				camera[i] = cv2.VideoCapture(i)
				if not(camera[i].isOpened()):
					camera[i].Open(i)
				boolean,frame[i] = camera[i].read()
				gray = cv2.cvtColor(frame[i], cv2.COLOR_BGR2GRAY)
				if abs(int(np.sum(gray)) - int(np.sum(Oldframe[i]))) < np.sum(gray)/20:  
					control[i] = True
					print "frame uguali nella camera " + str(i)
				else:
					print "frame diversi nella camera " + str(i)
					control[i] = False
					Oldframe[i] = gray
				camera[i].release()
			#WB + SAVE
			if(control.count(True)==nc):
				print 'White Balance'
				for j in range(0,nc):
					cv2.imwrite("immagini_fotocamere/camera_" + str(j) + "_in" + ".png", frame[j])
					frame[j] = edge(frame[j]);
					cv2.imwrite("immagini_fotocamere/camera_" + str(j) + "_out" + ".png", frame[j])

					flag = False
				#3D RECOSTRUCTION
				print '3D recostruction'
				disparityMap = main(frame[0],frame[1])
				cv2.imwrite("immagini_fotocamere/disparityMap.png", disparityMap)
			
			oggetto = True #stato oggetto
			
		
		
	


	#GO ARDUINO
		ard.write('G') # G = GO ARDUINO

	
		

	if "OFF" in x:
		oggetto = False
		
	if "SCATTA" in x:
		
	#STOP ARDUINO
		ard.write('S') # S = STOP ARDUINO
		
		for i in range(0,nc): 
			camera[i] = cv2.VideoCapture(i)
			if not(camera[i].isOpened()):
				camera[i].Open(i)
			boolean,frame[i] = camera[i].read()
			#WB + SAVE
			print 'White Balance'
			cv2.imwrite("immagini_fotocamere/camera_" + str(i) + "_in_SCATTA" + ".png", frame[i])		
			frame[i] = edge(frame[i]);			
			cv2.imwrite("immagini_fotocamere/camera_" + str(i) + "_out_SCATTA" + ".png", frame[i])
			camera[i].release()

		#3D RECOSTRUCTION
		print '3D recostruction'
		disparityMap = main(frame[0],frame[1])
		cv2.imwrite("immagini_fotocamere/disparityMap.png", disparityMap)
		
		
		
	

	#GO ARDUINO
		ard.write('G') # G = GO ARDUINO
		
		
arduino.close() #chiude la comunicazione con arduino


