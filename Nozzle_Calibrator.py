import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage
from PyQt5 import uic
from PyQt5 import QtGui
import cv2, imutils
import numpy as np
import math
from matplotlib import pyplot as plt
import logging
import winsound

import queue
from Nozzle_Serial import *

camera_type = 2                    # 1 : 1920x1080, 2 : 3840x2160
nozzle1_shift = 250*1*camera_type  # nozzle1 shift pixels to calculate weight

#UI파일 연결
#단, UI파일은 Python 코드 파일과 같은 디렉토리에 위치해야한다.
form_class = uic.loadUiType("Nozzle_Calibrator.ui")[0]


#화면을 띄우는데 사용되는 Class 선언
class WindowClass(QMainWindow, form_class) :
    def __init__(self) :
        super().__init__()
        self.setupUi(self)  # To use Qt Designer .ui file

        self.SerialCommDlg = SerialDialog()

        #버튼에 기능을 연결하는 코드
        self.pushButton_Open1.clicked.connect(self.loadImage)
        self.pushButton_Open1_1.clicked.connect(self.loadImage)
        self.pushButton_Open2.clicked.connect(self.loadImage)
        self.pushButton_Detect1.clicked.connect(self.Detect)
        self.pushButton_Detect1_1.clicked.connect(self.Detect)
        self.pushButton_Detect2.clicked.connect(self.Detect)
        self.pushButton_Save.clicked.connect(self.savePhoto)
        self.radioButton_Pattern.clicked.connect(self.matchingType)
        self.radioButton_Circle.clicked.connect(self.matchingType)

        self.toolButton_Up.clicked.connect(self.set_RIO_Offset)
        self.toolButton_Down.clicked.connect(self.set_RIO_Offset)
        self.toolButton_Left.clicked.connect(self.set_RIO_Offset)
        self.toolButton_Right.clicked.connect(self.set_RIO_Offset)

        self.pushButton_Serial.clicked.connect(self.serial_comm)
                
        self.Step = 0
        self.MatchingType = "pattern"   # 0:Pattern Matching, 1:Circle Matching
        self.filename = None # Will hold the image address location
        self.RIO_img = None # Will hold the temporary image for display
        self.ROI = {'name':'ROI', 'x':920, 'y':450  , 'size': 200}
        self.ROI_offset = {'name':'ROI_offset', 'x':0, 'y':0}
        self.ROI_hole = {'name':'ROI_hole', 'x':0, 'y':0}
        self.Matching_Offset = {'name':'Matching_Offset', 'x':0, 'y':0, 'mouse_clicked':0}

        self.nozzle_position = {'name':'nozzle_position', 'N1_x':0, 'N1_y':0, 'N1_1_x':0, 'N1_1_y':0, 'N2_x':0, 'N2_y':0} # base on entire image
        self.nozzle_offset = {'name':'nozzle_offset', 'x':0, 'y':0}
        self.weight = {'name':'weight', 'x_diff':0, 'y_diff':0, 'Weight':0.0}  # 0.02 (10mm = 500 pixel)
        
    def serial_comm(self):
        self.SerialCommDlg.show()

    def initParam(self):
        print('initParam')
        self.ROI = {'name':'ROI', 'x':920, 'y':450, 'size': 200}
        self.ROI_offset = {'name':'ROI_offset', 'x':0, 'y':0}
        self.ROI_hole = {'name':'ROI_hole', 'x':0, 'y':0}
        self.nozzle_position = {'name':'nozzle_position', 'N1_x':0, 'N1_y':0, 'N1_1_x':0, 'N1_1_y':0, 'N2_x':0, 'N2_y':0} # base on entire image
        self.nozzle_offset = {'name':'nozzle_offset', 'x':0, 'y':0}
        self.ROI['x'] = camera_type*self.ROI['x']
        self.ROI['y'] = camera_type*self.ROI['y']
        self.ROI['size'] = camera_type*self.ROI['size']

    def loadImage(self):
        print('loadImage')
        print(self.ROI, self.ROI_offset, self.ROI_hole, self.nozzle_position, self.nozzle_offset, sep='\n')

        # Get Image From File
        if self.checkBox_Camera.isChecked() == True:
            if self.getFileNameFromCamera(0) == True:
                self.filename = "captured.png"
            else:
                self.filename = 0
        else:
            self.filename = QFileDialog.getOpenFileName(filter="Image (*.*)")[0]
        print('file name1 : ', self.filename)

        if self.filename:
            self.updateStep()
            self.readImage()
            self.reloadROI()

            # Check Image Size
            if self.checkImgSize() == True:
                # Set offset
                if self.Step == 3:
                    self.ROI_offset = {'name':'ROI_offset', 'x':-nozzle1_shift, 'y':0}
                else:
                    self.ROI_offset = {'name':'ROI_offset', 'x':0, 'y':0}

                # Crop Image into ROI            
                StartY = self.ROI['y'] + self.ROI_offset['y']
                EndY = StartY + self.ROI['size']
                StartX = self.ROI['x'] + self.ROI_offset['x']
                EndX = StartX + self.ROI['size']
                self.RIO_img = self.image[StartY:EndY, StartX:EndX]
                
                # Camera Image Display
                self.showImage(self.image)
                
                # ROI Image Display
                self.showROI(self.RIO_img)
                
                # Information Display
                self.updateInfo()
                self.updateProgressBar()
            else:
                QMessageBox.warning(self, "message", "Check Camera Resolution or Image Size!!")
        else:
            QMessageBox.warning(self, "message", "User Cancel")
    
    def checkImgSize(self):
        print('checkImgSize')
        H, W, Ch = self.image.shape
        print('ImgSize : ', W, H)
        if W != 1920*camera_type or H != 1080*camera_type:
            QMessageBox.warning(self, "message", "Not supporte Image Size : {0}x{1} ".format(W, H))
            return False
        else:
            return True

    def drawRectangle(self, image):
        print('drawRectangle')
        cv2.rectangle(image,
            (self.ROI['x']+self.ROI_offset['x'], self.ROI['y']+self.ROI_offset['y']),
            (self.ROI['x']+self.ROI['size']+self.ROI_offset['x'], self.ROI['y']+self.ROI['size']+self.ROI_offset['y']),
            (0, 0, 255),
            4,    # fill : -1
            8)

    def showROI(self, image):
        print('showROI')
        # ROI display
        frame_ROI = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        frame_ROI = imutils.resize(frame_ROI, width=400)
        image = QImage(frame_ROI, frame_ROI.shape[1], frame_ROI.shape[0], frame_ROI.strides[0], QImage.Format_RGB888)
        self.label_ROI.setPixmap(QtGui.QPixmap.fromImage(image))

    def showImage(self, image):
        print('showImage')
        #QMessageBox.about(self, "message", "showImage")
        """ This function will take image input and resize it 
            only for display purpose and convert it to QImage
            to set at the label.
        """
        # Camera Image display
        self.drawRectangle(image)
        image = imutils.resize(image, width=300)
        frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
        self.label_Camera.setPixmap(QtGui.QPixmap.fromImage(image))

    def readImage(self):
        print('readImage')
        """ This function will read the image file
        """
        # Image
        ff = np.fromfile(self.filename, np.uint8)
        self.image = cv2.imdecode(ff, cv2.IMREAD_UNCHANGED)
        #self.image = cv2.imread(self.filename)

    def reloadROI(self):
        print('reloadROI')
        """ This function will reload the ROI image from self.image
        """
        # ROI
        StartY = self.ROI['y'] + self.ROI_offset['y']
        EndY = StartY + self.ROI['size']
        StartX = self.ROI['x'] + self.ROI_offset['x']
        EndX = StartX + self.ROI['size']
        self.RIO_img = self.image[StartY:EndY, StartX:EndX]

    def savePhoto(self):
        print('savePhoto')
        """ This function will save the image"""
        filename = QFileDialog.getSaveFileName(filter="JPG(*.jpg);;PNG(*.png);;TIFF(*.tiff);;BMP(*.bmp)")[0]

        cv2.imwrite(filename, self.RIO_img)
        print('Image saved as:', filename)

    def Detect(self):
        print('Detect')
        self.updateStep()
        image = self.image
        ROI_img = self.RIO_img

        if self.checkBox_Manual.isChecked() == True:
            hist1 = cv2.calcHist([ROI_img],[0],None,[256],[0,256])
            plt.subplot(221),plt.imshow(image,'gray'),plt.title('Entire Image')
            plt.subplot(222),plt.imshow(ROI_img,'gray'),plt.title('RIO Iamge')
            plt.subplot(223),plt.plot(hist1,color='r')
            plt.xlim([0,256])
            plt.connect('button_press_event', self.plot_eventHandle)
            plt.show(block=False)
            self.hide()    
        else:
            self.hide()
            if self.MatchingType == "pattern":
                self.patternMatching()
            else:
                self.circleMatching()
            self.show()
            self.updateNozzleInfo()
            self.displayHole(self.RIO_img)
            self.updateProgressBar()
            self.updateInfo()

    def plot_eventHandle(self, event):
        print('plot_eventHandle')
        # button 1: 마우스 좌클릭
        if event.button == 1:
            print('plt left clicked', int(event.xdata), int(event.ydata))

        # button 3: 마우스 우클릭 시 기존 입력값 삭제
        if event.button == 3:
            print('plt right clicked', int(event.xdata), int(event.ydata))
            self.ROI_hole['x'] = int(event.xdata)
            self.ROI_hole['y'] = int(event.ydata)

            self.updateNozzleInfo()
            plt.close()
            self.show()
            self.displayHole(self.RIO_img)
            self.updateProgressBar()
            self.updateInfo()

    def updateNozzleInfo(self):
        print('updateNozzleInfo')
        if self.Step == 2:
            self.nozzle_position['N1_x'] = self.ROI_hole['x'] + self.ROI['x'] + self.ROI_offset['x']
            self.nozzle_position['N1_y'] = self.ROI_hole['y'] + self.ROI['y'] + self.ROI_offset['y']
        elif self.Step == 4:
            self.nozzle_position['N1_1_x'] = self.ROI_hole['x'] + self.ROI['x'] + self.ROI_offset['x']
            self.nozzle_position['N1_1_y'] = self.ROI_hole['y'] + self.ROI['y'] + self.ROI_offset['y']
            self.weight['x_diff'] = self.nozzle_position['N1_x'] - self.nozzle_position['N1_1_x']
            print('x_diff : ', self.weight['x_diff'])
            self.weight['y_diff'] = self.nozzle_position['N1_y'] - self.nozzle_position['N1_1_y']
            print('y_diff : ', self.weight['y_diff'])
            self.weight['Weight'] = 5 / math.sqrt( math.pow(self.weight['x_diff'], 2) + math.pow(self.weight['y_diff'], 2) )
            print('y_diff : ', self.weight['Weight'])
        elif self.Step == 6:
            self.nozzle_position['N2_x'] = self.ROI_hole['x'] + self.ROI['x'] + self.ROI_offset['x']
            self.nozzle_position['N2_y'] = self.ROI_hole['y'] + self.ROI['y'] + self.ROI_offset['y']
            self.nozzle_offset['x'] = self.nozzle_position['N2_x'] - self.nozzle_position['N1_x']
            self.nozzle_offset['y'] = self.nozzle_position['N2_y'] - self.nozzle_position['N1_y']

    def displayHole(self, image):
        # line
        print('displayHole')
        cv2.line(self.RIO_img, (self.ROI_hole['x'], 0), (self.ROI_hole['x'], self.ROI['size']), (0, 0, 255), 1)
        cv2.line(self.RIO_img, (0, self.ROI_hole['y']), (self.ROI['size'], self.ROI_hole['y']), (0, 0, 255), 1)
        # cross
        if self.Step == 6:
            # nozzle 1
            nozzle_x = self.ROI_hole['x'] - self.nozzle_offset['x'] 
            nozzle_y = self.ROI_hole['y'] - self.nozzle_offset['y'] 
            cv2.line(self.RIO_img, (nozzle_x-5, nozzle_y), (nozzle_x+5, nozzle_y), (0, 255, 0), 2)
            cv2.line(self.RIO_img, (nozzle_x, nozzle_y-5), (nozzle_x, nozzle_y+5), (0, 255, 0), 2)
            # nozzle 2
            nozzle_x = self.ROI_hole['x']
            nozzle_y = self.ROI_hole['y']
            cv2.line(self.RIO_img, (nozzle_x-5, nozzle_y), (nozzle_x+5, nozzle_y), (0, 255, 0), 2)
            cv2.line(self.RIO_img, (nozzle_x, nozzle_y-5), (nozzle_x, nozzle_y+5), (0, 255, 0), 2)
        self.showROI(self.RIO_img)

    def updateInfo(self):
        print('updateInfo')
        path    = self.filename
        H, W, Ch = self.image.shape
        imgInfo = 'W:{0}  H:{1}  Ch:{2}'.format(W, H, Ch)
        ROIInfo = 'Start_x:{0}  Start_y:{1}  Offset_X:{2}  Offset_y:{3}'.format(self.ROI['x'], self.ROI['y'], self.ROI_offset['x'], self.ROI_offset['y'])
        nozzle_position1 = '(x:{0} y:{1})'.format(self.nozzle_position['N1_x'], self.nozzle_position['N1_y'])
        nozzle_position1_1 = '(x:{0} y:{1})'.format(self.nozzle_position['N1_1_x'], self.nozzle_position['N1_1_y'])
        nozzle_position2 = '(x:{0} y:{1})'.format(self.nozzle_position['N2_x'], self.nozzle_position['N2_y'])
        nozzle_offset_pxl = '(x:{0} y:{1})'.format(self.nozzle_offset['x'], self.nozzle_offset['y'])
        nozzle_offset_mm = '(x:{0:.3f} y:{1:.3f})'.format(self.weight['Weight']*self.nozzle_offset['x'], self.weight['Weight']*self.nozzle_offset['y'])
        weight = '{0:.3f} (x_diff:{1}, y_diff:{2})'.format(self.weight['Weight'], self.weight['x_diff'], self.weight['y_diff'])
        
        self.SerialCommDlg.N2_offset['x'] = self.weight['Weight']*self.nozzle_offset['x']
        self.SerialCommDlg.N2_offset['y'] = -self.weight['Weight']*self.nozzle_offset['y']
        sub_result = '(x : {0:.3f} y : {1:.3f})'.format(self.weight['Weight']*self.nozzle_offset['x'], -self.weight['Weight']*self.nozzle_offset['y'])
        main_result = '(x : {0:.1f} y : {1:.1f})'.format(10*round(self.SerialCommDlg.N2_offset['x'], 1), 10*round(self.SerialCommDlg.N2_offset['y'], 1))

        # File Path
        self.label_FileName.setText(path)
        # Size, Color
        self.label_ImgInfo.setText(imgInfo)
        # ROI position
        self.label_ROI_Position.setText(ROIInfo)
        # Nozzle Position
        self.label_Nozzle_Position1.setText(nozzle_position1)
        self.label_Nozzle_Position1_1.setText(nozzle_position1_1)
        self.label_Nozzle_Position2.setText(nozzle_position2)
        # Result Offset
        self.label_Nozzle_Offset_pxl.setText(nozzle_offset_pxl)
        self.label_Nozzle_Offset_mm.setText(nozzle_offset_mm)
        self.label_Weight.setText(weight)
        self.label_Result.setText(main_result)
        self.SerialCommDlg.label_Result.setText(sub_result)
        
        if self.Step == 6:
            self.label_Nozzle_Position1.setStyleSheet("Color : blue")
            self.label_Nozzle_Position1_1.setStyleSheet("Color : blue")
            self.label_Nozzle_Position2.setStyleSheet("Color : blue")
            self.label_Nozzle_Offset_pxl.setStyleSheet("Color : green")
            self.label_Nozzle_Offset_mm.setStyleSheet("Color : green")
            self.label_Weight.setStyleSheet("Color : green")
            self.label_Result.setStyleSheet("Color : red")
        else:
            self.label_Nozzle_Position1.setStyleSheet("Color : black")
            self.label_Nozzle_Position1_1.setStyleSheet("Color : black")
            self.label_Nozzle_Position2.setStyleSheet("Color : black")
            self.label_Nozzle_Offset_pxl.setStyleSheet("Color : black")
            self.label_Nozzle_Offset_mm.setStyleSheet("Color : black")
            self.label_Weight.setStyleSheet("Color : black")
            self.label_Result.setStyleSheet("Color : black")

    def updateStep(self):
        print('self.Step (before):', self.Step)

        # Check multiple Detect() call
        if (self.Step == 2 and self.sender().objectName() == 'pushButton_Detect1') or \
           (self.Step == 4 and self.sender().objectName() == 'pushButton_Detect1_1') or \
           (self.Step == 6 and self.sender().objectName() == 'pushButton_Detect2'):
           self.readImage()
           self.reloadROI()
            
        # Check Sequence Violation
        if self.sender().objectName() == 'pushButton_Open1':
            self.Step = 1
            self.initParam()
        elif self.sender().objectName() == 'pushButton_Detect1':
            if self.Step == 1 or self.Step == 2:
                self.Step = 2
            else:
                QMessageBox.warning(self, "message", "Sequence Violation (Previous Step : {0}), try again from Step1".format(self.Step))
                self.Step = 0
        elif self.sender().objectName() == 'pushButton_Open1_1':
            if self.Step == 2 or self.Step == 3 or self.Step == 4:
                self.Step = 3
            else:
                QMessageBox.warning(self, "message", "Sequence Violation (Previous Step : {0}), try again from Step1".format(self.Step))
                self.Step = 0
        elif self.sender().objectName() == 'pushButton_Detect1_1':
            if self.Step == 3 or self.Step == 4:
                self.Step = 4
            else:
                QMessageBox.warning(self, "message", "Sequence Violation (Previous Step : {0}), try again from Step1".format(self.Step))
                self.Step = 0
        elif self.sender().objectName() == 'pushButton_Open2':
            if self.Step == 4 or self.Step == 5 or self.Step == 6:
                self.Step = 5
            else:
                QMessageBox.warning(self, "message", "Sequence Violation (Previous Step : {0}), try again from Step1".format(self.Step))
                self.Step = 0
        elif self.sender().objectName() == 'pushButton_Detect2':
            if self.Step == 5 or self.Step == 6:
                self.Step = 6
            else:
                QMessageBox.warning(self, "message", "Sequence Violation (Previous Step : {0}), try again from Step1".format(self.Step))
                self.Step = 0
        else:
            QMessageBox.warning(self, "message", "Undefined Step Num : {0}, try again at the beginning".format(self.Step))
            self.Step = 0
        print('self.Step (after):', self.Step)

    def updateProgressBar(self):
        self.progressBar.setValue((100/6)*self.Step)
        print(self.Step, self.ROI, self.ROI_offset, self.ROI_hole, self.nozzle_position, self.nozzle_offset, sep='\n')

    def set_RIO_Offset(self):
        print('set_RIO_Offset : ', self.sender().objectName())

        if self.Step == 1 or self.Step == 3 or self.Step == 5:
            if self.sender().objectName() == 'toolButton_Up':
                self.ROI_offset['y'] -= 50
            elif self.sender().objectName() == 'toolButton_Down':
                self.ROI_offset['y'] += 50
            elif self.sender().objectName() == 'toolButton_Left':
                self.ROI_offset['x'] -= 50
            elif self.sender().objectName() == 'toolButton_Right':
                self.ROI_offset['x'] += 50
            else:
                QMessageBox.warning(self, "message", "Undefined Offset direction, try again at the beginning")
        else:
            QMessageBox.warning(self, "message", "ROI position can be adjusted only Step1,3,5")

        # Camera Image Display
        self.readImage()
        self.reloadROI()
        self.drawRectangle(self.image)
        self.showImage(self.image)
        self.showROI(self.RIO_img)
        
        # Update Image Info
        self.updateInfo()

    def matchingType(self):
        Key = 0
        if self.radioButton_Pattern.isChecked():
            self.MatchingType = "pattern"
            print('Matching : pattern')
        else:
            self.MatchingType = "circle"
            print('Matching : circle')

    def getFileNameFromCamera(self, id):
        capture = cv2.VideoCapture(id, cv2.CAP_DSHOW)
        if capture.isOpened():
            print ('isOpened OK')
        else:
            print ('isOpened NG')
            QMessageBox.critical(self, "message", "Fail to open camera")
            return False

        # Get Info
        print('Frame width:', int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print('Frame height:', int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print('Frame count:', int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
        print('FPS:', int(capture.get(cv2.CAP_PROP_FPS)))

        # Setup
        #capture.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        #capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
        #capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        #capture.set(cv2.CAP_PROP_FPS, 10)

        print ('Show Frame')

        while True:
            
            ret, frame = capture.read()  # blank frame in case of high resolution
            ret, frame = capture.read()  # normal frame
            if frame is None:
                print ('Fail to read frame!!')
                break
            
            W = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            H = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        
            if int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) > 640:
                img = imutils.resize(frame, width=1280)
                cv2.putText(img, 'Capture mode :'+str(W)+'x'+str(H), (10, 30), \
                            cv2.FONT_HERSHEY_PLAIN, 2.0, (0,0,255), 1, cv2.LINE_AA)
                cv2.putText(img, '  - Enter key : Capture Screen', (10, 60), \
                            cv2.FONT_HERSHEY_PLAIN, 1.5, (0,0,255), 1, cv2.LINE_AA)
                cv2.putText(img, '  - Esc : Exit', (10, 90), \
                            cv2.FONT_HERSHEY_PLAIN, 1.5, (0,0,255), 1, cv2.LINE_AA)    
                cv2.imshow('VideoCapture', img)
            else:
                cv2.putText(frame, 'Video mode :'+str(W)+'x'+str(H), (10, 30), \
                            cv2.FONT_HERSHEY_PLAIN, 2.0, (0,255,0), 1, cv2.LINE_AA)
                cv2.putText(frame, '  - Spacebar : Capture mode', (10, 60), \
                            cv2.FONT_HERSHEY_PLAIN, 1.5, (0,255,0), 1, cv2.LINE_AA)
                cv2.putText(frame, '  - Esc : Exit', (10, 90), \
                            cv2.FONT_HERSHEY_PLAIN, 1.5, (0,255,0), 1, cv2.LINE_AA)    
                cv2.imshow('VideoCapture', frame)

            key = cv2.waitKey(10)
            if key == 13:   # enter key
                if int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) > 640:
                    winsound.PlaySound("shutter.wav", winsound.SND_ASYNC)
                    cv2.imwrite('Captured.png',frame, params=[cv2.IMWRITE_PNG_COMPRESSION,0])
                    logging.debug ("Save done!!")
                    break
                else:
                    QMessageBox.critical(self, "message", "Try again in Capture mode")
                    continue

            if key == ord('q') or key == 27:
                logging.debug ("Exit without capture!!")
                break
            
            #if key == 27:   # esc key
            if key == 32:   # spacebar key
                capture.release()
                capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, 3840) # 3840 x 2160, 2592 x 1944, 2048 x 1536...640 x 480
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

                print('Frame width:', int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
                print('Frame height:', int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        
        print ('loop exit')

        if int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) > 640 and (key == 13) :
            result = True
        else:
            result = False
        
        capture.release()
        cv2.destroyAllWindows()

        return result

#######################################################################################
    def patternMatching(self):
        print('patternMatching')

        # 입력이미지와 템플릿 이미지 읽기
        img = self.RIO_img
        if camera_type == 1:
            template = cv2.imread('./Images/hole_22x22.jpg')
        else:
            template = cv2.imread('./Images/hole_44x44.jpg')

        if (type(template) is np.ndarray):
            th, tw = template.shape[:2]

            # 1가지 매칭 메서드
            methods = ['cv2.TM_CCOEFF_NORMED']
            # 3가지 매칭 메서드 순회
            # methods = ['cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR_NORMED', \
            #                                     'cv2.TM_SQDIFF_NORMED']
            
            for i, method_name in enumerate(methods):
                img_draw = img.copy()
                method = eval(method_name)
                # 템플릿 매칭   ---①
                res = cv2.matchTemplate(img, template, method)
                # 최대, 최소값과 그 좌표 구하기 ---②
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                print(method_name, min_val, max_val, min_loc, max_loc)

                # TM_SQDIFF의 경우 최소값이 좋은 매칭, 나머지는 그 반대 ---③
                if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                    top_left = min_loc
                    match_val = min_val
                else:
                    top_left = max_loc
                    match_val = max_val
                # 매칭 좌표 구해서 사각형 표시   ---④      
                bottom_right = (top_left[0] + tw, top_left[1] + th)
                cv2.rectangle(img_draw, top_left, bottom_right, (0,0,255), 1)
                # 매칭 포인트 표시 ---⑤
                cv2.putText(img_draw, str(match_val)[:5], top_left, \
                            cv2.FONT_HERSHEY_PLAIN, 0.8, (0,255,0), 1, cv2.LINE_AA)
                # cv2.imshow(method_name, img_draw)
                img_draw = imutils.resize(img_draw, width=800)
                text = "Center : X {0}, Y {1}".format(round((top_left[0]+bottom_right[0])/2), round((top_left[1]+bottom_right[1])/2))
                cv2.putText(img_draw, text, (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 1)
                cv2.imshow(method_name, img_draw)
                cv2.setMouseCallback(method_name, self.mouse_callback)

                self.Matching_Offset['x'] = self.Matching_Offset['y'] = self.Matching_Offset['mouse_clicked'] = 0
                shape_change = False

                # Save result
                self.ROI_hole['x'] = round((top_left[0]+bottom_right[0])/2) + self.Matching_Offset['x']
                self.ROI_hole['y'] = round((top_left[1]+bottom_right[1])/2) + self.Matching_Offset['y']

                while True: 
                    
                    key = cv2.waitKeyEx() 
                    print('key value : ', key)
                    if key == 0x1B: #ESC키 
                        break
                    elif key==0x270000: # 방향키 방향 전환 0x270000==right 
                        print('right direction key')
                        self.Matching_Offset['x'] += 1 
                    elif key==0x280000: # 방향키 방향 전환 0x280000==down 
                        print('down direction key')
                        self.Matching_Offset['y'] += 1 
                    elif key==0x250000: # 방향키 방향 전환 0x250000==left 
                        print('left direction key')
                        self.Matching_Offset['x'] -= 1 
                    elif key==0x260000: # 방향키 방향 전환 0x260000==up
                        print('up direction key')
                        self.Matching_Offset['y'] -= 1 
                    elif key==0x000020: # spacebar
                        print('spacebar')
                        shape_change = not shape_change
                    else:
                        print('undefined key : ')
                        break

                    if self.Matching_Offset['mouse_clicked'] == 1:  # manual position by mouse 
                        self.Matching_Offset['mouse_clicked'] = 0
                        wdith = bottom_right[0]-top_left[0]
                        height = bottom_right[1]-top_left[1]
                        top_left = (-round(wdith/2), -round(height/2))
                        bottom_right= (top_left[0]+wdith, top_left[1]+height)

                    img_draw = img.copy()
                    cv2.putText(img_draw, str(match_val)[:5], (top_left[0]+self.Matching_Offset['x'], top_left[1]+self.Matching_Offset['y']), \
                            cv2.FONT_HERSHEY_PLAIN, 0.8, (0,255,0), 1, cv2.LINE_AA)
                    if shape_change == False:
                        cv2.rectangle(img_draw, (top_left[0]+self.Matching_Offset['x'], top_left[1]+self.Matching_Offset['y']), \
                            (bottom_right[0]+self.Matching_Offset['x'], bottom_right[1]+self.Matching_Offset['y']), (0,0,255), 1)
                    else:
                        cv2.circle(img_draw, (round((top_left[0]+bottom_right[0])/2) + self.Matching_Offset['x'], \
                            round((top_left[1]+bottom_right[1])/2) + self.Matching_Offset['y']), 10, (0, 0, 255), 1) 
                    
                    img_draw = imutils.resize(img_draw, width=800)
                    text = "Center : X {0}, Y {1}".format(round((top_left[0]+bottom_right[0])/2) + self.Matching_Offset['x'], round((top_left[1]+bottom_right[1])/2) + self.Matching_Offset['y'])
                    cv2.putText(img_draw, text, (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 1)
                    cv2.imshow(method_name, img_draw)

                    # Update result
                    self.ROI_hole['x'] = round((top_left[0]+bottom_right[0])/2) + self.Matching_Offset['x']
                    self.ROI_hole['y'] = round((top_left[1]+bottom_right[1])/2) + self.Matching_Offset['y']
                    print('Pattern Matching Result : ', self.ROI_hole['x'], self.ROI_hole['y'])

                cv2.destroyAllWindows()
        else:
            QMessageBox.critical(self, "message", "Fail to load template file")

    def mouse_callback(self, event, x, y, flags, param): 
        img = self.RIO_img
        img_draw = img.copy() 
        mouse_is_pressing = False
        
        # 마우스 왼쪽 버튼 누를 시 발생 이벤트 
        if event == cv2.EVENT_LBUTTONDOWN: 
            mouse_is_pressing = True
            # Save result
            self.Matching_Offset['x'] = camera_type*round(x/4)
            self.Matching_Offset['y'] = camera_type*round(y/4)
            self.Matching_Offset['mouse_clicked'] = 1
            self.ROI_hole['x'] = self.Matching_Offset['x']
            self.ROI_hole['y'] = self.Matching_Offset['y']
            # draw cross
            cv2.line(img_draw, (self.Matching_Offset['x']-5, self.Matching_Offset['y']), (self.Matching_Offset['x']+5, self.Matching_Offset['y']), (0, 0, 255), 1)
            cv2.line(img_draw, (self.Matching_Offset['x'], self.Matching_Offset['y']-5), (self.Matching_Offset['x'], self.Matching_Offset['y']+5), (0, 0, 255), 1)
            img_draw = imutils.resize(img_draw, width=800)
            text = "Center : X {0}, Y {1}".format(self.Matching_Offset['x'], self.Matching_Offset['y'])
            cv2.putText(img_draw, text, (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 1)
            cv2.imshow('cv2.TM_CCOEFF_NORMED', img_draw)
            #cv2.imshow('test', img_draw)
            print('Mouse L button down :', x, y)

        # 마우스 이동시 발생하는 이벤트
        elif event == cv2.EVENT_MOUSEMOVE: 
            if mouse_is_pressing: # 마우스 이동하는 조건인 true 동안 rectangle 그리기 
                print('Mouse L button moving')

        # 마우스 왼쪽 버튼에서 손을 떼면 발생하는 이벤트
        elif event == cv2.EVENT_LBUTTONUP: 
            mouse_is_pressing = False
            print('Mouse L button up')

#######################################################################################
    def circleMatching(self):
        print('circleMatching')

        img = self.RIO_img

        # Non-local Meaning Denoising
        nmd = cv2.fastNlMeansDenoisingColored(img, dst=None, h=20, hColor=20, templateWindowSize=7, searchWindowSize=21)

        # color to grayscale
        gray = cv2.cvtColor(nmd, cv2.COLOR_BGR2GRAY)

        # Thresholding
        ret, thres = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Hough Circle Transform
        circles = cv2.HoughCircles(thres, cv2.HOUGH_GRADIENT, dp=1, minDist=3000, param1=100, param2=10, minRadius=5, maxRadius=20)
        if circles is not None:
            img_draw = img.copy()
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # 원 둘레에 초록색 원 그리기
                cv2.circle(img_draw, (i[0], i[1]), i[2], (0,255,0), 1)
                # 원 중심점에 빨강색 원 그리기
                cv2.circle(img_draw, (i[0], i[1]), 1, (0,0,255), 1)

        #cv2.putText(img, text, (circles[0][0][0], circles[0][0][1]+40), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 1)
        
        img_draw = imutils.resize(img_draw, width=800)
        text = "Center : X {0}, Y {1}, Radius {2}".format(circles[0][0][0], circles[0][0][1], circles[0][0][2])
        cv2.putText(img_draw, text, (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 1)
        cv2.imshow('nozzle center', img_draw)

        offset_h = offset_v = 0
        while True: 
            
            key = cv2.waitKeyEx() 
            if key == 0x1B: #ESC키 
                break
            elif key==0x270000: # 방향키 방향 전환 0x270000==right 
                print('right direction key')
                offset_h += 1 
            elif key==0x280000: # 방향키 방향 전환 0x280000==down 
                print('down direction key')
                offset_v += 1 
            elif key==0x250000: # 방향키 방향 전환 0x250000==left 
                print('left direction key')
                offset_h -= 1 
            elif key==0x260000: # 방향키 방향 전환 0x260000==up
                print('up direction key')
                offset_v -= 1 
            else:
                print('undefined key : ', key)
                break

            img_draw = img.copy()
            text = "Center : X {0}, Y {1}, Radius {2}".format(circles[0][0][0]+offset_h, circles[0][0][1]+offset_v, circles[0][0][2])
            
            for i in circles[0,:]:
                # 원 둘레에 초록색 원 그리기
                cv2.circle(img_draw, (i[0]+offset_h, i[1]+offset_v), i[2], (0,255,0), 1)
                # 원 중심점에 빨강색 원 그리기
                cv2.circle(img_draw, (i[0]+offset_h, i[1]+offset_v), 1, (0,0,255), 1)
            img_draw = imutils.resize(img_draw, width=800)
            cv2.putText(img_draw, text, (10, 30), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0,0,255), 1)
            cv2.imshow('nozzle center', img_draw)

        # Save result
        self.ROI_hole['x'] = circles[0][0][0] + offset_h
        self.ROI_hole['y'] = circles[0][0][1] + offset_v
        print('Circle Matching Result : ', self.ROI_hole['x'], self.ROI_hole['y'])
        cv2.destroyAllWindows()
#######################################################################################

if __name__ == "__main__" :
    app = QApplication(sys.argv)
    myWindow = WindowClass() 
    myWindow.show()
    app.exec_()

# https://gozz123.tistory.com/29
# https://github-wiki-see.page/m/8BitsCoding/RobotMentor/wiki/python_pyserial
# https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=chandong83&logNo=221156763486
# https://m.blog.naver.com/rhukjin/222040320807
# https://wikidocs.net/16035 byte화
# https://python.hotexamples.com/examples/serial/Serial/flush/python-serial-flush-method-examples.html
# https://enjoytools.net/xe/board_PZRP31/5125 thread 이용