import numpy as np
import cv2
import csv
import matplotlib.pyplot as plt

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float32 x
property float32 y
property float32 z
comment property uchar red    {canale R} non funzionano su paraviewer
comment property uchar green  {canale G}
comment property uchar blue   {canale B}
element face 0
property list uint8 int32 vertex_indices
end_header
'''

def save(table,name):
	# write it
	with open(name+'.csv', 'w') as csvfile:
		writer = csv.writer(csvfile)
    	        [writer.writerow(r) for r in table]
def read(name):
	# read it
	with open(name+'.csv', 'r') as csvfile:
    		reader = csv.reader(csvfile)
    		table = [[float(e) for e in r] for r in reader]
	return table

def calibrareCamere(path_left,path_rigth,numero_im):
    print 'calibrareCamere'
    findCorner_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

    objp_pattern = np.zeros((6*7,3), np.float32)
    objp_pattern[:,:2] = np.mgrid[0:162:27,0:189:27].T.reshape(-1,2) # pattern con grandezza lato square

    obj_points = []
    img_left_points = []
    img_right_points = []

    for i in range(1,numero_im+1):

        gray_left = cv2.imread(path_left+'Left'+str(i)+'.jpg',0)
        #print type(gray_left)
        gray_rigth = cv2.imread(path_rigth+'Right'+str(i)+'.jpg',0)
        #print type(gray_rigth)
        image_size = gray_left.shape[::-1]

        find_chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK

        left_found, left_corners = cv2.findChessboardCorners(gray_left, (6,7), None)
        right_found, right_corners = cv2.findChessboardCorners(gray_rigth, (6,7), None)

        if left_found and right_found:
            cv2.cornerSubPix(gray_left, left_corners, (11,11), (-1,-1), findCorner_criteria)
            cv2.cornerSubPix(gray_rigth, right_corners, (11,11), (-1,-1), findCorner_criteria)
            img_left_points.append(left_corners)
            img_right_points.append(right_corners)
            obj_points.append(objp_pattern)

            '''
            cv2.drawChessboardCorners(gray_left, (7,6), left_corners, left_found)
            cv2.drawChessboardCorners(gray_rigth, (7,6), right_corners, right_found)
            cv2.imshow("left chess", gray_left)
            cv2.imshow("right chess", gray_rigth)
            cv2.waitKey(0)
            '''

    ret, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(obj_points, img_left_points, image_size, None, None)
    ret, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(obj_points, img_right_points, image_size, None, None)

    tot_error_R = 0
    for i in xrange(len(obj_points)):
        imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecsR[i], tvecsR[i], mtxR, distR)
        error = cv2.norm(img_right_points[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        tot_error_R += error

    tot_error_L = 0
    for i in xrange(len(obj_points)):
        imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecsL[i], tvecsL[i], mtxL, distL)
        error = cv2.norm(img_left_points[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        tot_error_L += error

    print "mean error left: ", tot_error_L/len(obj_points)
    print "mean error right: ", tot_error_R/len(obj_points)

    return (mtxL,distL,mtxR,distR,obj_points,img_left_points,img_right_points)

def undistortImage(imgLeft,imgRigth,mtxL,distL,mtxR,distR):

    hL,  wL = imgLeft.shape[:2]
    hR, wR = imgRigth.shape[:2]

    new_camera_matrix_Left, roiLeft = cv2.getOptimalNewCameraMatrix(mtxL,distL,(wL,hL),0,None)
    new_camera_matrix_Rigth, roiRigth = cv2.getOptimalNewCameraMatrix(mtxR,distR,(wR,hR),0,None)


    #stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
    #stereocalib_flags = (cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_SAME_FOCAL_LENGTH
                        #| cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_USE_INTRINSIC_GUESS)

    LeftDist = cv2.undistort(imgLeft, mtxL, distL, None, new_camera_matrix_Left)
    RigthDist = cv2.undistort(imgRigth, mtxR, distR, None, new_camera_matrix_Rigth)

    mtxL = new_camera_matrix_Left
    mtxR = new_camera_matrix_Rigth

    xL,yL,wL,hL = roiLeft
    xR,yR,wR,hR = roiRigth

    Left = False
    Right = False

    if(roiLeft!=(0,0,0,0)):
        LeftDist = LeftDist[yL:yL+hL, xL:xL+wL]
        #cv2.imshow("LeftDist", LeftDist)
        #cv2.waitKey(0)
        Left = True
    if(roiRigth!=(0,0,0,0)):
        RigthDist = RigthDist[yR:yR+hR, xR:xR+wR]
        #cv2.imshow("RigthDist", RigthDist)
        #cv2.waitKey(0)
        Right = True
    if(Left==True and Right==True):
        print "DISTORTE ENTRAMBI"
        return (LeftDist,RigthDist)
    if(Left==True and Right==False):
        print "DISTORTA SINISTRA"
        return (LeftDist,imgRigth)
    if(Left==False and Right==True):
        print "DISTORTA DESTRA"
        return (imgLeft,RigthDist)
    if(Left==False and Right==False):
        print "DISTORTA NESSUNA"
        return (imgLeft,imgRigth)

def rettificaImmagini(imgLeft,imgRigth,mtxL,distL,mtxR,distR,image_size,R,T):

    R1 = np.zeros((3*3,1))
    R2 = np.zeros((3*3,1))
    P1 = np.zeros((3*4,1))
    P2 = np.zeros((3*4,1))
    Q = None

    R1,R2,P1,P2,Q,roi1,roi2 = cv2.stereoRectify(mtxL, distL, mtxR, distR, image_size,
                                                R, T, flags =  cv2.CALIB_ZERO_DISPARITY, alpha =-1)

    # Q = matrice che serve nel metodo reprojectImageTo3D, per trovare disparity #
    # R1,R2 = matrici di rotazione per la prima e seconda camera, per rettificare immagini #
    # P1,P2 = matrici di proiezione nel nuovo sistema per la prima e seconda camera #

    map1x, map1y = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, image_size, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, image_size, cv2.CV_32FC1)

    imgLeftRemap = cv2.remap(imgLeft, map1x, map1y, cv2.INTER_LANCZOS4)
    imgRigthRemap = cv2.remap(imgRigth, map2x, map2y, cv2.INTER_LANCZOS4)


    insiemeRett = np.hstack([imgLeftRemap, imgRigthRemap])
    insiemeNoRett = np.hstack([imgLeft, imgRigth])
    for line in range(0, int(insiemeRett.shape[0] / 20)):
        insiemeRett[line * 20, :] = (0)
    for line in range(0, int(insiemeNoRett.shape[0] / 20)):
        insiemeNoRett[line * 20, :] = (0)
    
    plt.imshow(insiemeNoRett, 'gray')
    plt.show()

    plt.imshow(insiemeRett,'gray')
    plt.show()
    
    '''
    xL,yL,wL,hL = roi1
    print roi1
    xR,yR,wR,hR = roi2
    print roi2

    imgLeftRemap = imgLeftRemap[yL:yL+hL, xL:xL+wL]
    imgRigthRemap = imgRigthRemap[yR:yR+hR, xR:xR+wR]
    print imgLeftRemap.shape
    print imgRigthRemap.shape
    '''
    '''
    for line in range(0, int(imgLeft.shape[0] / 20)):
        imgLeft[line * 20, :] = (0)
    for line in range(0, int(imgRigth.shape[0] / 20)):
        imgRigth[line * 20, :] = (0)
    '''

    return (imgLeftRemap,imgRigthRemap,Q)

def stereoMatch(imgLeft,imgRight):

    #stereo Correspondence
    window_size = 3
    minDisp = 25
    numDisp = 112
    blockSize = 5
    P1 = 8*2*blockSize*2
    P2 = 32*2*blockSize**2
    disp12MaxDiff = 1
    preFilterCap = 50
    uniquenessRatio = 5
    speckleWindowSize = 100
    speckleRange = 2

    stereo = cv2.StereoSGBM_create(minDisparity=minDisp,numDisparities=numDisp, blockSize=blockSize,
                                    P1=P1, P2=P2, disp12MaxDiff=disp12MaxDiff,preFilterCap=preFilterCap,uniquenessRatio=uniquenessRatio,
                                    speckleWindowSize=speckleWindowSize,speckleRange=speckleRange, mode=False)
    disparitySGBM = stereo.compute(imgLeft,imgRight)
    disparitySGBM = (disparitySGBM - minDisp) / 16
    #plt.imshow(disparitySGBM,'gray')
    #plt.show()

    #filtro di smoth
    disparity = cv2.medianBlur(disparitySGBM,5)
    plt.imshow(disparity,'gray')
    plt.show()

    return(disparity)

def pointCloud(disp, im, imgRigth, mtxL, distL, mtxR, distR, image_size, R, T, Q):

    im1 = im[:,:,0] #B
    im2 = im[:,:,1] #G
    im3 = im[:,:,2] #R

    points = cv2.reprojectImageTo3D(disp, Q) #punti(X Y Z)

    im1, imgRightRemap, Q = rettificaImmagini(im1,imgRigth,mtxL, distL, mtxR, distR, image_size, R, T)
    im2, imgRightRemap, Q = rettificaImmagini(im2,imgRigth,mtxL, distL, mtxR, distR, image_size, R, T)
    im3, imgRightRemap, Q = rettificaImmagini(im3,imgRigth,mtxL, distL, mtxR, distR, image_size, R, T)

    im = cv2.merge([im1, im2, im3]) #ho l'immagine rettificata in BGR

    colors = cv2.cvtColor(im, cv2.COLOR_BGR2RGB) #punti( B G R)

    mask = disp > 0 # maggiore di dove la disp e -1, sconosciuta
    coords = points[mask]
    colors = colors[mask]
    coords = coords.reshape(-1,3)
    colors = colors.reshape(-1,3)
    verts = np.hstack([coords, colors])
    '''con questo su ogni riga del file ho poszione vertice e colore, no bene con paraviewer	
    with open('out.ply', 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d'), %f,%d tipo di dati
    '''

    with open('out.ply', 'wb') as f:
       	f.write((ply_header % dict(vert_num=len(coords))).encode('utf-8'))
        np.savetxt(f, coords, fmt='%f %f %f')
