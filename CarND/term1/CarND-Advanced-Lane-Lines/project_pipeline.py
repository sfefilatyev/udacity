import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import os
import pickle
import copy
from moviepy.editor import VideoFileClip
from IPython.display import HTML

# Constants
LANE_WIDTH = 3.70 # lane with in meters
INTER_FRAME_WINDOW = 7 # the number of frame to average lane parameters from

# and Global variables
prev_left_fit = None
prev_right_fit = None

def calibrate_camera(filename='calibration.pickle', display_test=False, display_corners=False):
    """Routine to extract camera calibration and distortion parameters."""
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    if os.path.exists(filename):
        with open(filename,'rb') as file:
            object = pickle.load(file)
            mtx = object['camera_mtx']
            dist = object['distortion']
    else:
        n_rows = 6
        n_columns = 9
        objp = np.zeros((n_rows*n_columns,3), np.float32)
        objp[:,:2] = np.mgrid[0:n_columns,0:n_rows].T.reshape(-1,2)
        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.

        # Make a list of calibration images
        images = glob.glob('./camera_cal/calibration*.jpg')
        gray = 0
        # Step through the list and search for chessboard corners
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (n_columns,n_rows),None)
            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)
                if display_corners:
                    # Draw and display the corners
                    img = cv2.drawChessboardCorners(img, (n_columns,n_rows), corners, ret)
                    cv2.imshow('img',img)
                    cv2.waitKey(27)
    
        if display_corners:
            cv2.destroyAllWindows()
        print('Using {} views of chessboard for camera calibration'.format(len(objpoints)))
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        # loading a test distorted image to show how we can undistort artifactes after computing calibation parameters
        if not ret:
            return None
        else:
            with open(filename,'wb') as file:
                object = {}
                object['camera_mtx'] = mtx
                object['distortion'] = dist
                pickle.dump(object,file)
        
    if display_test:
        test_img = cv2.imread('./camera_cal/calibration1.jpg')
        test_img = cv2.cvtColor(test_img, cv2.COLOR_BGR2RGB)
        undistorted = cv2.undistort(test_img, mtx, dist, None, mtx)
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
        f.tight_layout()
        ax1.imshow(test_img)
        ax1.set_title('Original Image', fontsize=50)
        ax2.imshow(undistorted)
        ax2.set_title('Undistorted Image', fontsize=50)
        plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
    
    
    return mtx, dist

def threshold_image(image, display_color=False):
    """Threshold image using S-channel of HLS and magnitude of gradient."""
    if type(image) is str:
        image = cv2.imread(image)
        # for debuggin puporses converting it to RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # as well as getting gray and HLS
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    R = image[:,:,0]
    H = hls[:,:,0]
    L = hls[:,:,1]
    S = hls[:,:,2]
    
    thresh = (200, 255)
    binary_r = np.zeros_like(R)
    binary_r[(R > thresh[0]) & (R <= thresh[1])] = 1
    
    thresh = (170, 255)
    binary_s = np.zeros_like(S)
    binary_s[(S > thresh[0]) & (S <= thresh[1])] = 1
    
    thresh = (15, 100)
    binary_h = np.zeros_like(H)
    binary_h[(H > thresh[0]) & (H <= thresh[1])] = 1
    
    # Sobel x
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away 
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    
    # Threshold x gradient
    thresh_min = 20
    thresh_max = 100
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1
    
    # Combine the two binary thresholds
    combined_binary = np.zeros_like(sxbinary)
    combined_binary[(binary_s == 1) | (sxbinary == 1)] = 1

    if display_color:
        # Stack each channel
        # Note color_binary[:, :, 0] is all 0s, effectively an all black image. It might
        # be beneficial to replace this channel with something else.
        color_binary = np.dstack(( np.zeros_like(sxbinary), sxbinary, binary_s))*255
        return combined_binary, color_binary
    else:
        return combined_binary
    

def get_transform_params():
    # since the images are supplied in a single resolution I hardcoded the values for
    # perspective transform
    src_points = np.float32([[580, 460], [203, 720], [1127, 720], [705, 460]])
    dst_points = np.float32([[320, 0], [320, 720], [960, 720], [960, 0]])
    #src_points = np.float32([[605, 445], [203, 720], [1127, 720], [675, 445]])
    #dst_points = np.float32([[320, 0], [320, 720], [960, 720], [960, 0]])
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    M_inv = cv2.getPerspectiveTransform(dst_points, src_points)
    return M, M_inv

def make_perspective_transform(image, camera_matrix, dist_coeff, M, display_polygon=False):
    """Transform a perspective image into a birds-eye-view image."""
    
    if type(image) is str:
        image = cv2.imread(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # rectifying the original image
    undistorted_img = cv2.undistort(image, camera_matrix, dist_coeff, None, camera_matrix)
    img_size = (undistorted_img.shape[1], undistorted_img.shape[0])
    # and calculating apprpriate perspective transform from source points to dest points
    warped = cv2.warpPerspective(undistorted_img, M, img_size)
    
    if display_polygon:
        src_points = np.float32([[580, 460], [203, 720], [1127, 720], [705, 460]])
        dst_points = np.float32([[320, 0], [320, 720], [960, 720], [960, 0]])
        warped_clone = copy.deepcopy(warped)
        cv2.fillPoly(warped_clone, np.int_([dst_points]), (0,255, 0))
        warped = cv2.addWeighted(warped, 1, warped_clone, 0.3, 0)
        undistored_clone = copy.deepcopy(undistorted_img)
        cv2.fillPoly(undistored_clone, np.int_([src_points]), (0,255, 0))
        result = cv2.addWeighted(undistorted_img, 1, undistored_clone, 0.3, 0)
        return warped, result
    return warped
    
    
def warp_thresholded(image, camera_mtx, dist_coeff, M):
    """Thresholds a color-image using S-of-HLS channel and magnitude of gradient"""
    
    binary_image = threshold_image(image)
    binary_warped = make_perspective_transform(binary_image, camera_mtx, dist_coeff, M)
    return binary_warped

def draw_lanes(image, camera_mtx, dist_coeff, M_inv, pts_left, pts_right, lane_pts_left, lane_pts_right):
    # Create an image to draw the lines on
    color_warp = np.zeros_like(image, dtype=np.uint8)
    undist = cv2.undistort(image, camera_mtx, dist_coeff, None, camera_mtx)
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_right = np.flipud(pts_right) # flipping top and bottom sides
    pts = np.concatenate((pts_left, pts_right))
    
    # Draw the lane onto the warped blank image
    #color_warp[lane_pts_left[:,1], lane_pts_left[:,0],:]=(255,0,0)
    #color_warp[lane_pts_right[:,1], lane_pts_right[:,0], :]=(0,0,255)
    color_warp = cv2.polylines(color_warp, [pts_left], False, [255,0,0], 15)
    color_warp = cv2.polylines(color_warp, [pts_right], False, [0,0,255], 15)
    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
    
    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, M_inv, (image.shape[1], image.shape[0])) 
    # Combine the result with the original image
    result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
    return result


def calculate_curvature(ploty, leftx, rightx, avg_distance_btw_lanes):
    """Calculates curvature of lane-lines closest to the ego-vehicle"""
    
    # Define y-value where we want radius of curvature
    # I'll choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)
    
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30/720 # meters per pixel in y dimension
    xm_per_pix = 3.7/avg_distance_btw_lanes # meters per pixel in x dimension - average distance between lanes is 3.7
    
    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters
    radius = (left_curverad + right_curverad)//2 

    return radius


def fit_lanes(nonzerox, nonzeroy, left_lane_inds, right_lane_inds, binary_warped, midpoint):
    """Extracts position of left and right lane pixels and parameters of fitted lines.
    
    The method is used by both find_lanes_from_scratch() and find_lanes_continue()
    """
    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 
    
    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # Generatign lane coordinates in birds eye view
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    # Obtaining statistics on the lane
    fit_diff = right_fitx - left_fitx # correspnding distance between left and right lane pixels
    avg_distance_btw_lanes = np.mean(fit_diff) # average lane width in pixels
    midpoint_btw_lanes = (right_fitx[-1]+left_fitx[-1])/2 # central line in the ego lane

    distance_from_center = - (midpoint - midpoint_btw_lanes) # ego distance from central line in ego lane in pixels
    distance_from_center_normalized = distance_from_center/avg_distance_btw_lanes # now, normalized by the lane width
    distance_from_center_metric = distance_from_center_normalized*LANE_WIDTH # ego distance from the central line in metric units
    
    pts_left = np.concatenate((np.int32([left_fitx]),np.int32([ploty])), axis=0)
    pts_left = np.transpose(pts_left)
    pts_right = np.concatenate((np.int32([right_fitx]),np.int32([ploty])), axis=0)
    pts_right = np.transpose(pts_right)
    
    # Extract left and right line pixel positions
    lane_pts_left = np.concatenate((np.int_([leftx]),np.int_([lefty])), axis=0)
    lane_pts_left = np.transpose(lane_pts_left)
    lane_pts_right = np.concatenate((np.int_([rightx]),np.int_([righty])), axis=0)
    lane_pts_right = np.transpose(lane_pts_right)
    
    # calculating curvature
    radius = calculate_curvature(ploty, left_fitx, right_fitx, avg_distance_btw_lanes)
    
    return left_fit, right_fit, pts_left, pts_right, lane_pts_left, lane_pts_right, avg_distance_btw_lanes, distance_from_center_metric, radius


def display_lane(image, 
                 out_img, 
                 nonzerox, 
                 nonzeroy, 
                 left_lane_inds, 
                 right_lane_inds, 
                 pts_left, 
                 pts_right, 
                 lane_pts_left, 
                 lane_pts_right, 
                 distance_from_center_metric, 
                 radius, 
                 camera_mtx, 
                 dist_coeff, 
                 M_inv):
    # Generate x and y values for plotting
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    out_img = cv2.polylines(out_img, [pts_left], False, [255,255,255], 5)
    out_img = cv2.polylines(out_img, [pts_right], False, [255,255,255], 5)
    if type(image) is str:
        image = cv2.imread(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    out_img_original = draw_lanes(image, camera_mtx, dist_coeff, M_inv, pts_left, pts_right, lane_pts_left, lane_pts_right)
    #drawing curvature and center position
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(out_img_original,'Distance from center: {0:.2f} m'.format(distance_from_center_metric),(30,30), font, 1,(255,0,0),2,cv2.LINE_AA)
    cv2.putText(out_img_original,'Curvature radius: {} m'.format(radius),(640,30), font, 1,(255,0,0),2,cv2.LINE_AA)
    return out_img_original, out_img
        

def find_lanes_from_scratch(image, camera_mtx, dist_coeff, M, M_inv, display=False):
    """Finds exact ego-lanes for the vehicles"""
    
    binary_warped = warp_thresholded(image, camera_mtx, dist_coeff, M)
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    if display:
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    
    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int(binary_warped.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    
    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        
        if display:
            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
        
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
            
    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    
    left_fit, right_fit, pts_left, pts_right, lane_pts_left, lane_pts_right, avg_distance_btw_lanes, distance_from_center_metric, radius = \
        fit_lanes(nonzerox, nonzeroy, left_lane_inds, right_lane_inds, binary_warped, midpoint)

    if display: 
        out_img_original, out_img = display_lane(image, 
                                                 out_img, 
                                                 nonzerox, 
                                                 nonzeroy, 
                                                 left_lane_inds, 
                                                 right_lane_inds, 
                                                 pts_left, 
                                                 pts_right, 
                                                 lane_pts_left, 
                                                 lane_pts_right, 
                                                 distance_from_center_metric, 
                                                 radius, 
                                                 camera_mtx, 
                                                 dist_coeff, 
                                                 M_inv)
        return (left_fit, right_fit, pts_left, pts_right, avg_distance_btw_lanes, distance_from_center_metric, 
                radius, out_img_original, out_img)
    else:
        return (left_fit, right_fit, pts_left, pts_right, avg_distance_btw_lanes, distance_from_center_metric, radius)
    

def find_lanes_continue(image, camera_mtx, dist_coeff, M, M_inv, left_fit, right_fit, display=False, binary_warped=None):
    """Finds exact ego-lanes for the vehicles using parameters of lines
       from previoius frames."""
    if not binary_warped: # if not provided from a previous procedure
        binary_warped = warp_thresholded(image, camera_mtx, dist_coeff, M)
    if display:
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    
    midpoint = binary_warped.shape[1]//2
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & \
                      (nonzerox < (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin))) 
    right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & \
                       (nonzerox < (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))  
    left_fit, right_fit, pts_left, pts_right, lane_pts_left, lane_pts_right, avg_distance_btw_lanes, distance_from_center_metric, radius = \
        fit_lanes(nonzerox, nonzeroy, left_lane_inds, right_lane_inds, binary_warped, midpoint)
        
    if display: 
        out_img_original, out_img = display_lane(image, 
                                                 out_img, 
                                                 nonzerox, 
                                                 nonzeroy, 
                                                 left_lane_inds, 
                                                 right_lane_inds, 
                                                 pts_left, 
                                                 pts_right, 
                                                 lane_pts_left, 
                                                 lane_pts_right, 
                                                 distance_from_center_metric, 
                                                 radius, 
                                                 camera_mtx, 
                                                 dist_coeff, 
                                                 M_inv)
        return (left_fit, right_fit, pts_left, pts_right, avg_distance_btw_lanes, distance_from_center_metric, 
                radius, out_img_original, out_img)
    else:
        return (left_fit, right_fit, pts_left, pts_right, avg_distance_btw_lanes, distance_from_center_metric, radius)


def process_image(image):
    global prev_left_fit, prev_right_fit
    image=image.astype('uint8')
    #checking if the image is in the range of (0,255)
    if(image.max()<1):
        image=image*255
    if prev_left_fit is None:
        (left_fit, right_fit, pts_left, pts_right, avg_distance_btw_lanes, distance_from_center_metric, 
                radius, out_img_original, out_img) = find_lanes_from_scratch(image, camera_mtx, dist_coeff, M, M_inv, display=True)
    else:
        (left_fit, right_fit, pts_left, pts_right, avg_distance_btw_lanes, distance_from_center_metric, 
                radius, out_img_original, out_img) = find_lanes_continue(image, camera_mtx, dist_coeff, M, M_inv, prev_left_fit, prev_right_fit, display=True)            
    
    if avg_distance_btw_lanes>750 or avg_distance_btw_lanes < 550:
        print ("Too large/too small lane width - searching from scratch")
        (left_fit, right_fit, pts_left, pts_right, avg_distance_btw_lanes, distance_from_center_metric, 
                radius, out_img_original, out_img) = find_lanes_from_scratch(image, camera_mtx, dist_coeff, M, M_inv, display=True)
    prev_left_fit = left_fit
    prev_right_fit = right_fit
    
    return out_img_original

camera_mtx, dist_coeff = calibrate_camera(display_test=False)
M,M_inv = get_transform_params()
#warped_image, display_img = make_perspective_transform('./test_images/straight_lines1.jpg', camera_mtx, dist_coeff, M, display_polygon=False)
#warped_image, display_img = make_perspective_transform('./test_images/test1.jpg', camera_mtx, dist_coeff, M, display_polygon=True)
#left_fit, right_fit, left_fitx, right_fitx, out_img = find_lanes_from_scratch('./test_images/straight_lines1.jpg', camera_mtx, dist_coeff, M, M_inv, display=True)
(left_fit, right_fit, pts_left, pts_right, avg_distance_btw_lanes, distance_from_center_metric, 
                radius, out_img_original, out_img) = find_lanes_from_scratch('./test_images/test6.jpg', camera_mtx, dist_coeff, M, M_inv, display=True)
    
video_input = './project_video.mp4'
video_output = './project_video_output.mp4'
#video_input = './challenge_video.mp4'
#video_output = './challenge_video_output.mp4'
#video_input = './harder_challenge_video.mp4'
#video_output = './harder_challenge_video_output.mp4'
clip1 = VideoFileClip(video_input)
clip1_output = clip1.fl_image(process_image) #NOTE: this function expects color images!!
clip1_output.write_videofile(video_output, audio=False)

