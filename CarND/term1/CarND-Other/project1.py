#importing some useful packages
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2


#reading in an image
image = mpimg.imread('test_images/solidWhiteRight.jpg')
#printing out some stats and plotting
print('This image is:', type(image), 'with dimesions:', image.shape)
plt.imshow(image)  #call as plt.imshow(gray, cmap='gray') to show a grayscaled image

import math

def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    """
    NOTE: this is the function you might want to use as a starting point once you want to 
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).  
    
    Think about things like separating line segments by their 
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of 
    the lines and extrapolate to the top and bottom of the lane.
    
    This function draws `lines` with `color` and `thickness`.    
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    #for one lane line the slope will be positive, for the other, it will be negative.
    #The "negative" slope will average the values of all negative slopes
    #The "positive" slope will average the values of all positives slopes. 
    #To make the lanes uninterrupting we will record the max-top and min-bottom values of the original slopes and
    
    negative_slope=0.0
    negative_slope_count=0
    
    positive_slope=0.0
    positive_slope_count=0
    
    min_y=img.shape[0]
    max_y=img.shape[0]-1  #this value will be constant. It will define the bottom of image where solid lanes are drawn
    
    positive_slope_x_list=[]
    positive_slope_y_list=[]
    
    negative_slope_x_list=[]
    negative_slope_y_list=[]


    for line in lines:
        for x1,y1,x2,y2 in line:
            slope=((y2-y1)/(x2-x1))
            print ("Current slope:",slope)
            line_image = np.copy(img)*0
            cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
            plt.imshow(line_image)
            if slope>=0:
                #positive_slope=positive_slope+1
                #positive_slope_count=positive_slope_count+1
                
                positive_slope_x_list.append(x1)
                positive_slope_x_list.append(x2)
                positive_slope_y_list.append(y1)
                positive_slope_y_list.append(y2)
            else:
                #negative_slope=negative_slope+1
                #negative_slope_count=negative_slope_count+1
                
                negative_slope_x_list.append(x1)
                negative_slope_x_list.append(x2)
                negative_slope_y_list.append(y1)
                negative_slope_y_list.append(y2)
                
            if y1<min_y:
                min_y=y1
            if y2<min_y:
                min_y=y2
    
    #positive_slope=positive_slope/positive_slope_count
    #negative_slope=negative_slope/negative_slope_count
    
    positive_line=np.polyfit(positive_slope_x_list, positive_slope_y_list, 1)
    negative_line=np.polyfit(negative_slope_x_list, negative_slope_y_list, 1)
    
    #print positive_line
    #print negative_line
    
    #finding intersection of those lines with the horizonal lines from region of interest:
    #top(in the image) horizon line has the equation 0*x+Top_y, or parameters (0,Top_y) - slope =0
    #bottom(in the image) horizontal line has the equation 0*x+Bottom_y, or parameters (0,Bottom_y) slope=0
    top_horizontal_line=[0,min_y]
    bottom_horizontal_line=[0,max_y]
    
    positive_top_x=int((positive_line[1]-top_horizontal_line[1])*1.0/(top_horizontal_line[0]-positive_line[0]))
    positive_top_y=int(top_horizontal_line[1])
    print (positive_top_x,positive_top_y)
    
    negative_top_x=int((negative_line[1]-top_horizontal_line[1])*1.0/(top_horizontal_line[0]-negative_line[0]))
    negative_top_y=int(top_horizontal_line[1])
    print (negative_top_x,negative_top_y)
                
    positive_bottom_x=int((positive_line[1]-bottom_horizontal_line[1])*1.0/(bottom_horizontal_line[0]-positive_line[0]))
    positive_bottom_y=int(bottom_horizontal_line[1])
    print ("Positive_bottom_x=",positive_bottom_x,"Positive_bottom_y=",positive_bottom_y)
    
    negative_bottom_x=int((negative_line[1]-bottom_horizontal_line[1])*1.0/(bottom_horizontal_line[0]-negative_line[0]))
    negative_bottom_y=int(bottom_horizontal_line[1])    
    print (negative_bottom_x,negative_bottom_y)
        
    #and displaying just two lines - with positive slope and negative
    if (img.max()<1):
        img=img*256
        
    #img=img.astype('uint8')
    cv2.line(img, (positive_top_x, positive_top_y), (positive_bottom_x, positive_bottom_y), color, thickness)
    cv2.line(img, (negative_top_x, negative_top_y), (negative_bottom_x, negative_bottom_y), color, thickness)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap,thickness=2):
    """
    `img` should be the output of a Canny transform.
        
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape, 3), dtype=np.uint8)
    draw_lines(line_img, lines,thickness=thickness)
    return line_img



def process_image(image):
    # NOTE: The output you return should be a color image (3 channel) for processing video below
    # TODO: put your pipeline here,
    # you should return the final output (image with lines are drawn on lanes)
    
    #converting  the original image into grayscale
    image=image.astype('uint8')
    
    #checking if the image is in the range of (0,255)
    if(image.max()<1):
        image=image*256
    
    gray = grayscale(image)
    
    #blurring the grayscale image to remove spurious gradients
    gray_blurred=gaussian_blur(gray,5)
    
    #applying canny-edge detector to extract edges
    threshold_low=50
    threshold_high=150
    edge_image = canny(gray_blurred,threshold_low,threshold_high)
    
    #creating a mask for the region of interest (ROI)
    imshape = image.shape
    vertices = np.array([[(0,imshape[0]),(400, 335), (560, 335), (imshape[1],imshape[0])]], dtype=np.int32)
    ROI=region_of_interest(edge_image,vertices)
    
    #extractign hough lines from edges
    rho = 1 # distance resolution in pixels of the Hough grid
    theta = np.pi/180 # angular resolution in radians of the Hough grid
    threshold = 50     # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 150 #minimum number of pixels making up a line
    max_line_gap = 90    # maximum gap in pixels between connectable line segments
    line_image = hough_lines(ROI, rho, theta, threshold, min_line_length, max_line_gap)

    result = line_image
    return result

result=process_image(image)
plt.imshow(result)