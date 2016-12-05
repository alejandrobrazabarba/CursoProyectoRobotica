import numpy
import cv2

cap = cv2.VideoCapture(0)

img_width = 640
img_height = 480

# Define the regions of the image to analyse
NUM_REGIONS = 10
reg_vert_offset = 400
reg_vert_width = 20
reg_vert_end = reg_vert_offset + reg_vert_width
reg_horiz_width = img_width/NUM_REGIONS
reg_horiz_divs = numpy.arange(0,img_width+1,img_width/NUM_REGIONS)
reg_num_pixels = 128*20
intensity_threshold = 30

GREEN = (0,255,0)
RED = (0,0,255)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    print frame.shape
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # For each region of interest in the image
    for i in range(0,NUM_REGIONS):
        # Calculate average pixel intensity
        avg_int = numpy.median( gray[reg_vert_offset:reg_vert_end, reg_horiz_divs[i]:reg_horiz_divs[i+1]] )
        # Select color for rectangle depending on pixel intensity
        reg_rect_color = RED if avg_int < intensity_threshold else GREEN
        # Draw rectangle
        cv2.rectangle(frame, (reg_horiz_divs[i],reg_vert_offset),
                     (reg_horiz_divs[i+1]-1, reg_vert_end-1),reg_rect_color,1)

    # cv2.rectangle(frame, (330,0), (630,430),(0,255,0),3)
    # Histogram equalization
    # equR = cv2.equalizeHist(frame[:,:,2])
    # equG = cv2.equalizeHist(frame[:,:,1])
    # equB = cv2.equalizeHist(frame[:,:,0])
    # res = numpy.hstack((frame,cv2.merge((equB,equG,equR))))


    # Display the resulting frame come here
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
