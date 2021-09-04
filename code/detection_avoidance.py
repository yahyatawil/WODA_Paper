import cv2
import os
import sys, getopt
import signal
import time
import numpy as np
from edge_impulse_linux.image import ImageImpulseRunner

from pysabertooth import Sabertooth
import serial.tools.list_ports as port

"""
Settings
"""
show_camera = True
saftey_margin = 30 # margin width in pixels
AoI = np.array([[80,210],[240,210],[320-saftey_margin,320],[saftey_margin,320]]) # Trapezoid AoI
MARGIN_l = np.array([[0,0],[saftey_margin,0],[saftey_margin,320],[0,320]]) # left safty margin
MARGIN_r = np.array([[320-saftey_margin,0],[320,0],[320,320],[320-saftey_margin,320]]) # right safty margin

"""
Global variables
"""
runner = None
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG') , fps=2.0, frameSize=(320, 320))
prev_frame_time = 0
new_frame_time = 0


"""
Initialize the serial connection with sabertooth
return: saber object
"""
def initialization():
    print("\nDetecting sabertooth....\n")
    pl = list(port.comports())
    print(pl)
    address = ""
    for p in pl:
        print(p)
        if "Sabertooth" in str(p):
            address = str(p).split(" ")
    print("\nAddress found @")
    print(address[0])

    saber = Sabertooth(address[0], baudrate=9600, address=128, timeout=0.1)

    return saber

"""
Send velocity to motor 1 & 2
"""
def send_vel(w, saber):

    speed = 16

    if w < -0.05:
        print("left")
        saber.drive(1, speed)
        saber.drive(2, speed + 5*abs(w))
    elif w > 0.05:
        print("right")
        saber.drive(1, speed + 5*abs(w))
        saber.drive(2, speed)
    else:
        print("straight!")
        saber.drive(1, speed)
        saber.drive(2, speed)



"""
Draw bounding box arround detected object
"""
def drawBoundingBoxe(img, x1, y1, x2, y2, label, color):

    #color = (0,0,255)
    height = y2-y1
    width = x2-x1
    results = [
        {
            "left": x1,
            "top": y1,
            "width": width,
            "height": height,
            "label": label
        }
    ]

    for res in results:
        left = int(res['left'])
        top = int(res['top'])
        right = int(res['left']) + int(res['width'])
        bottom = int(res['top']) + int(res['height'])
        label = res['label']
        imgHeight, imgWidth, _ = img.shape
        thick = int((imgHeight + imgWidth) // 900)
        cv2.rectangle(img,(left, top), (right, bottom), color, 3)
        cv2.putText(img, label, (left, top - 12), 0, 1e-3 * imgHeight, color, thick)

"""
Draw feature point for obstacle (center)
"""
def draw_fp(img,x,y,w,h):
    fp_x = int(x) + int(w) # feature point x-axis
    fp_x = int(fp_x/2)
    fp_y = int(y) + int(h) # feature point y-axis
    fp_y = int(fp_y/2) 

    sp_x = 0 # safety point x-axis
    sp_y = fp_y # safety point y-axis

    if fp_x > 320/2: # feature point belongs to the left of image
        sp_x = int(320-(saftey_margin/2))
    else: # deature point belongs to the right of image
        sp_x = int(saftey_margin/2)

    cv2.circle(img, (sp_x,sp_y), radius=5, color=(255, 0, 0), thickness=-1) # draw a safty point
    cv2.circle(img, (fp_x,fp_y), radius=5, color=(0, 255, 0), thickness=-1) # draw feature point
    cv2.line(img,(fp_x,fp_y),(sp_x,sp_y),(255,0,0),3) # draw a line between them


"""
Check if the obstacle is in AoI.
return: -1.00 if not and 1.00 if yes
"""
def check_in_AoI(x,y,w,h):
    point_x = int(x)+ int(w)
    point_x = int(point_x /2 )
    point_y = int(y)+int(h)
    AoI_condition = cv2.pointPolygonTest(AoI, (point_x,point_y), False)
    return AoI_condition

def now():
    return round(time.time() * 1000)

"""
Connect to Raspberry PI camera
"""
def get_webcams():
    port_ids = []
    for port in range(5):
        print("Looking for a camera in port %s:" %port)
        camera = cv2.VideoCapture(port)
        if camera.isOpened():
            ret = camera.read()[0]
            if ret:
                backendName =camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) found in port %s " %(backendName,h,w, port))
                port_ids.append(port)
            camera.release()
    return port_ids

def sigint_handler(sig, frame):
    print('Interrupted')
    if (runner):
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

def help():
    print('missing <path_to_model.eim> <Camera port ID, only required when more than 1 camera is present>')

def main(argv):

    global prev_frame_time # used to calculate fps
    global new_frame_time # used to calculate fps

    try:
        opts, args = getopt.getopt(argv, "h", ["--help"])
    except getopt.GetoptError:
        help()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            help()
            sys.exit()

    if len(args) == 0:
        help()
        sys.exit(2)

    model = args[0]

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)

    #saber = initialization() # init saber object 

    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']
            if len(args)>= 2:
                videoCaptureDeviceId = int(args[1])
            else:
                port_ids = get_webcams()
                if len(port_ids) == 0:
                    raise Exception('Cannot find any webcams')
                if len(args)<= 1 and len(port_ids)> 1:
                    raise Exception("Multiple cameras found. Add the camera port ID as a second argument to use to this script")
                videoCaptureDeviceId = int(port_ids[0])

            camera = cv2.VideoCapture(videoCaptureDeviceId)
            ret = camera.read()[0]
            
            if ret:
                backendName = camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) in port %s selected." %(backendName,h,w, videoCaptureDeviceId))
                camera.release()
            else:
                raise Exception("Couldn't initialize selected camera.")


            for res, img in runner.classifier(videoCaptureDeviceId):
                new_frame_time = time.time()
                #print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))

                # iterate over the detected boxes
                for bb in res["result"]["bounding_boxes"]:
                    AoI_condition = check_in_AoI(bb['x'],bb['y'],bb['width'],bb['height'])
                    if AoI_condition > 0: # box entered the AoI
                        color= (0,255,0)                       
                        draw_fp(img,bb['x'],bb['y'],bb['width'],bb['height'])
                    else:
                        color = (255,0,0)
                    drawBoundingBoxe(img,bb['x'],bb['y'],bb['x']+bb['width'],bb['y']+bb['height'],'%s (%.2f)'%(bb['label'], bb['value']),color)
                    
                # Add the AoI to frame
                layer = np.zeros(img.shape,dtype=np.uint8)
                cv2.fillPoly(layer, pts = [AoI], color =(0,255,0))
                img = cv2.addWeighted(img, 0.9, layer, 0.1, 0.0)

                # Add the safty margin in yellow to image. 
                layer2 = np.zeros(img.shape,dtype=np.uint8)
                cv2.fillPoly(layer2, pts = [MARGIN_l], color =(0,255,255))
                cv2.fillPoly(layer2, pts = [MARGIN_r], color =(0,255,255))
                img = cv2.addWeighted(img, 0.9, layer2, 0.1, 0.0) 
                
                # add divider vertical line
                cv2.line(img,(int(320/2),0),(int(320/2),320),(255,255,255),1) # draw a line between them

                fps = 1/(new_frame_time-prev_frame_time)
                prev_frame_time = time.time()
                cv2.putText(img, "fps:{}".format(round(fps,2)), (40, 10), 0, 1e-3 * 320, (0,255,0), 1)
                
                out.write(img)


                if (show_camera):
                    cv2.imshow('edgeimpulse', img)
                    if cv2.waitKey(1) == ord('q'):
                        out.release()
                        break

        finally:
            if (runner):
                runner.stop()

if __name__ == "__main__":
   main(sys.argv[1:])