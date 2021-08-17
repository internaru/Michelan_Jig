import cv2, imutils
import logging
import winsound

logging_format = '%(levelname)s: %(asctime)s: %(message)s'
logging.basicConfig(level=logging.DEBUG, format=logging_format)  # Debug, Info, Warning, Error, Critical
logger = logging.getLogger()

# create a file handler
handler = logging.FileHandler('log.txt')
# create a logging format
formatter = logging.Formatter(logging_format)
handler.setFormatter(formatter)
# add the handlers to the logger
logger.addHandler(handler)

# http://www.webcamerausb.com/elp-4k-3840x2160-usb-webcam-manual-varifocus-lens-mini-case-sony-imx317-industrial-machine-vision-mini-usb-webcam-camera-p-338.html

logging.debug ('Opening Video Device')

capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)    # fast but blink problem
#capture = cv2.VideoCapture(0, cv2.CAP_MSMF)    # too slow but no blink problem
if capture.isOpened():
    logging.debug ('isOpened OK')
else:
    logging.debug ('isOpened NG')

# Setup
#capture.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
#capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
#capture.set(cv2.CAP_PROP_FRAME_WIDTH, 2592)
#capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1944)
#capture.set(cv2.CAP_PROP_FRAME_WIDTH, 2048)
#capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1536)
#capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#capture.set(cv2.CAP_PROP_BUFFERSIZE, 3)
#capture.set(cv2.CAP_PROP_FPS, 30)

# Get Info
print('Frame width:', int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
print('Frame height:', int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
print('Frame count:', int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
print('FPS:', int(capture.get(cv2.CAP_PROP_FPS)))

logging.debug ('Show Frame')

while True:
    
    ret, frame = capture.read()
    ret, frame = capture.read()
    if frame is None:
        logging.debug ("no frame")
        continue
    else:
        logging.debug ("frame")
    img = imutils.resize(frame, width=640)
    cv2.imshow('VideoCapture', img)
    #cv2.imshow('VideoCapture', frame)
    
    key = cv2.waitKey(10)
    if key == ord('q') or key == 27:
        #if int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) == 3840:
        if int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) > 640:
        #if int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)) == 1280:
            cv2.imwrite('Captured.png',frame, params=[cv2.IMWRITE_PNG_COMPRESSION,0])
            logging.debug ("Save done!!")
        break
    
    if key == 13: #
        winsound.PlaySound("shutter.wav", winsound.SND_ASYNC)
        capture.release()
        capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
        #capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        #capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
        #capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        #capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        capture.set(cv2.CAP_PROP_FPS, 2)

        print('Frame width:', int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print('Frame height:', int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print('Frame count:', int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
        print('FPS:', int(capture.get(cv2.CAP_PROP_FPS)))

logging.debug ('loop exit')

capture.release() 
cv2.destroyAllWindows()