import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2
from skimage.feature import hog
import glob
import tqdm
import time
from sklearn.svm import LinearSVC
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
import os
import sys
import pickle
from scipy.ndimage.measurements import label
import threading
from moviepy.editor import VideoFileClip, VideoClip
import copy
import queue as Queue
from numpy.distutils.misc_util import general_source_directories_files
import logging

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-9s) %(message)s',)

# Global parameters used for pipeline        
ystart = 370
ystop = 656
scales = [1.2, 1.6, 2.2] 
#scales =[1.6, 2.2]
color_space = 'YCrCb' # Can be RGB, HSV, LUV, HLS, YUV, YCrCb
orient = 9  # HOG orientations
pix_per_cell = 8 # HOG pixels per cell
cell_per_block = 2 # HOG cells per block
hog_channel = 'ALL' # Can be 0, 1, 2, or "ALL"
spatial_size = (32, 32) # Spatial binning dimensions
hist_bins = 128    # Number of histogram bins
spatial_feat = False # Spatial features on or off
hist_feat = False # Histogram features on or off
hog_feat = True # HOG features on or off
y_start_stop = [ystart, ystop] # Min and max in y to search in slide_window()
prev_img = None #variable to hold the previous image (needed in tracking)
prev_labels = None #variable to hold the previous label-map (needed in tracking)
NUM_THREADS = 8  # the number of parallel jobs
BUF_SIZE = 4 # The number of images in the queue
HEAT_THRESHOLD = 2

def convert_color(img, conv='RGB2YCrCb'):
    """ Convert the color space of the image"""
    if conv == 'RGB2YCrCb':
        return cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
    if conv == 'BGR2YCrCb':
        return cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    if conv == 'RGB2LUV':
        return cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
    if conv == 'RGB2HSV':
        return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    if conv == 'BGR2HSV':
        return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 
def get_hog_features(img, orient, pix_per_cell, cell_per_block, 
                        vis=False, feature_vec=True):
    """Define a function to return HOG features and visualization"""
    # Call with two outputs if vis==True
    if vis == True:
        features, hog_image = hog(img, 
                                  orientations=orient, 
                                  pixels_per_cell=(pix_per_cell, pix_per_cell),
                                  cells_per_block=(cell_per_block, cell_per_block), 
                                  transform_sqrt=True, 
                                  visualise=vis, feature_vector=feature_vec)
        return features, hog_image
    # Otherwise call with one output
    else:      
        features = hog(img, 
                       orientations=orient, 
                       pixels_per_cell=(pix_per_cell, pix_per_cell),
                       cells_per_block=(cell_per_block, cell_per_block), 
                       transform_sqrt=True, 
                       visualise=vis, feature_vector=feature_vec)
        return features


def bin_spatial(img, size=(32, 32)):
    """Define a function to compute binned color features """
    # Use cv2.resize().ravel() to create the feature vector
    features = cv2.resize(img, size).ravel() 
    # Return the feature vector
    return features


def color_hist(img, nbins=32, bins_range=(0, 256)):
    """Define a function to compute color histogram features """
    # NEED TO CHANGE bins_range if reading .png files with mpimg!
    # Compute the histogram of the color channels separately
    channel1_hist = np.histogram(img[:,:,0], bins=nbins, range=bins_range)
    channel2_hist = np.histogram(img[:,:,1], bins=nbins, range=bins_range)
    channel3_hist = np.histogram(img[:,:,2], bins=nbins, range=bins_range)
    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0]))
    # Return the individual histograms, bin_centers and feature vector
    return hist_features


def extract_features(imgs, color_space='RGB', spatial_size=(32, 32),
                        hist_bins=32, orient=9, 
                        pix_per_cell=8, cell_per_block=2, hog_channel=0,
                        spatial_feat=True, hist_feat=True, hog_feat=True):
    """Define a function to extract features from a list of images"""
    # Have this function call bin_spatial() and color_hist()
    # Create a list to append feature vectors to
    features = []
    # Iterate through the list of images
    for file in tqdm.tqdm(imgs):
        file_features = []
        # Read in each one by one
        image = mpimg.imread(file)
        # apply color conversion if other than 'RGB'
        if color_space != 'RGB':
            if color_space == 'HSV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            elif color_space == 'LUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2LUV)
            elif color_space == 'HLS':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
            elif color_space == 'YUV':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
            elif color_space == 'YCrCb':
                feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2YCrCb)
        else: feature_image = np.copy(image)      

        if spatial_feat == True:
            spatial_features = bin_spatial(feature_image, size=spatial_size)
            file_features.append(spatial_features)
        if hist_feat == True:
            # Apply color_hist()
            hist_features = color_hist(feature_image, nbins=hist_bins)
            file_features.append(hist_features)
        if hog_feat == True:
        # Call get_hog_features() with vis=False, feature_vec=True
            if hog_channel == 'ALL':
                hog_features = []
                for channel in range(feature_image.shape[2]):
                    hog_features.append(get_hog_features(feature_image[:,:,channel], 
                                        orient, pix_per_cell, cell_per_block, 
                                        vis=False, feature_vec=True))
                hog_features = np.ravel(hog_features)        
            else:
                hog_features = get_hog_features(feature_image[:,:,hog_channel], orient, 
                            pix_per_cell, cell_per_block, vis=False, feature_vec=True)
            # Append the new feature vector to the features list
            file_features.append(hog_features)
        features.append(np.concatenate(file_features))
    # Return list of feature vectors
    return features

def draw_boxes(img, bboxes, color=(0, 0, 255), thick=6):
    """Define a function to draw bounding boxes"""
    # Make a copy of the image
    imcopy = np.copy(img)
    # Iterate through the bounding boxes
    for bbox in bboxes:
        # Draw a rectangle given bbox coordinates
        cv2.rectangle(imcopy, bbox[0], bbox[1], color, thick)
    # Return the image copy with boxes drawn
    return imcopy


def single_img_features(img, color_space='RGB', spatial_size=(32, 32),
                        hist_bins=32, orient=9, 
                        pix_per_cell=8, cell_per_block=2, hog_channel=0,
                        spatial_feat=True, hist_feat=True, hog_feat=True):    
    """Define a function to extract features from a single image window"""
    # This function is very similar to extract_features()
    # just for a single image rather than list of images
    #1) Define an empty list to receive features
    img_features = []
    #2) Apply color conversion if other than 'RGB'
    if color_space != 'RGB':
        if color_space == 'HSV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        elif color_space == 'LUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
        elif color_space == 'HLS':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        elif color_space == 'YUV':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
        elif color_space == 'YCrCb':
            feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
    else: feature_image = np.copy(img)      
    #3) Compute spatial features if flag is set
    if spatial_feat == True:
        spatial_features = bin_spatial(feature_image, size=spatial_size)
        #4) Append features to list
        img_features.append(spatial_features)
    #5) Compute histogram features if flag is set
    if hist_feat == True:
        hist_features = color_hist(feature_image, nbins=hist_bins)
        #6) Append features to list
        img_features.append(hist_features)
    #7) Compute HOG features if flag is set
    if hog_feat == True:
        if hog_channel == 'ALL':
            hog_features = []
            for channel in range(feature_image.shape[2]):
                hog_features.extend(get_hog_features(feature_image[:,:,channel], 
                                    orient, pix_per_cell, cell_per_block, 
                                    vis=False, feature_vec=True))      
        else:
            hog_features = get_hog_features(feature_image[:,:,hog_channel], orient, 
                        pix_per_cell, cell_per_block, vis=False, feature_vec=True)
        #8) Append features to list
        img_features.append(hog_features)

    #9) Return concatenated array of features
    return np.concatenate(img_features)


def search_windows(img, windows, clf, scaler, color_space='RGB', 
                    spatial_size=(32, 32), hist_bins=32, 
                    hist_range=(0, 256), orient=9, 
                    pix_per_cell=8, cell_per_block=2, 
                    hog_channel=0, spatial_feat=True, 
                    hist_feat=True, hog_feat=True):
    """Define a function you will pass an image"""
    # and the list of windows to be searched (output of slide_windows())
        #1) Create an empty list to receive positive detection windows
    on_windows = []
    #2) Iterate over all windows in the list
    for window in windows:
        #3) Extract the test window from original image
        test_img = cv2.resize(img[window[0][1]:window[1][1], window[0][0]:window[1][0]], (64, 64))      
        #4) Extract features for that window using single_img_features()
        features = single_img_features(test_img, color_space=color_space, 
                            spatial_size=spatial_size, hist_bins=hist_bins, 
                            orient=orient, pix_per_cell=pix_per_cell, 
                            cell_per_block=cell_per_block, 
                            hog_channel=hog_channel, spatial_feat=spatial_feat, 
                            hist_feat=hist_feat, hog_feat=hog_feat)
        #5) Scale extracted features to be fed to classifier
        test_features = scaler.transform(np.array(features).reshape(1, -1))
        #6) Predict using your classifier
        prediction = clf.predict(test_features)
        #7) If positive (prediction == 1) then save the window
        if prediction == 1:
            on_windows.append(window)
    #8) Return windows for positive detections
    return on_windows

def add_heat(heatmap, bbox_list):
    """Iterate through list of bboxes to construct heatmap"""
    for box in bbox_list:
        # Add += 1 for all pixels inside each bbox
        # Assuming each "box" takes the form ((x1, y1), (x2, y2))
        heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += 1

    # Return updated heatmap
    return heatmap

def apply_threshold(heatmap, threshold):
    """Threshold heatmap by a value"""
    # Zero out pixels below the threshold
    heatmap[heatmap <= threshold] = 0
    # Return thresholded map
    return heatmap

def draw_labeled_bboxes(img, labels, draw_image = True):
    """Return a copy of the original with the cars drawn"""
    # Iterate through all detected cars
    if not draw_image:
        return img
    for car_number in range(1, labels[1]+1):
        # Find pixels with each car_number label value
        if (labels[0]==car_number).sum() == 0: # for the case we have already supressed the component
            continue
        nonzero = (labels[0] == car_number).nonzero()
        # Identify x and y values of those pixels
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Define a bounding box based on min/max x and y
        bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
        # Draw the box on the image
        cv2.rectangle(img, bbox[0], bbox[1], (0,0,255), 6)
    # Return the image
    return img


def find_cars(img, output_windows, ystart, ystop, scale, svc, X_scaler, orient, pix_per_cell, cell_per_block, hog_channel, spatial_size, hist_bins,  spatial_feat, hist_feat, hog_feat):
    """# Single function that can extracts features using hog sub-sampling, histogram, and spatial features to make predictions
    # returns a heat mat"""
    draw_img = np.copy(img)
    img = img.astype(np.float32)/255
    
    img_tosearch = img[ystart:ystop,:,:]
    ctrans_tosearch = convert_color(img_tosearch, conv='RGB2YCrCb')
    if scale != 1:
        imshape = ctrans_tosearch.shape
        ctrans_tosearch = cv2.resize(ctrans_tosearch, (np.int(imshape[1]/scale), np.int(imshape[0]/scale)))
        
    ch1 = ctrans_tosearch[:,:,0]
    ch2 = ctrans_tosearch[:,:,1]
    ch3 = ctrans_tosearch[:,:,2]

    # Define blocks and steps as above
    nxblocks = (ch1.shape[1] // pix_per_cell)-1
    nyblocks = (ch1.shape[0] // pix_per_cell)-1 
    nfeat_per_block = orient*cell_per_block**2
    # 64 was the orginal sampling rate, with 8 cells and 8 pix per cell
    window = 64
    nblocks_per_window = (window // pix_per_cell)-1 
    cells_per_step = 1  # Instead of overlap, define how many cells to step
    nxsteps = (nxblocks - nblocks_per_window) // cells_per_step
    nysteps = (nyblocks - nblocks_per_window) // cells_per_step
    
    # Compute individual channel HOG features for the entire image
    if hog_feat:
        
        if hog_channel == 'ALL':
            hog_features = []
            for channel in range(ctrans_tosearch.shape[2]):
                hog_features.append(get_hog_features(ctrans_tosearch[:,:,channel], 
                                    orient, pix_per_cell, cell_per_block, 
                                    vis=False, feature_vec=False))
            #hog_features = np.ravel(hog_features)        
        else:
            hog_features = get_hog_features(ctrans_tosearch[:,:,hog_channel], orient, 
                        pix_per_cell, cell_per_block, vis=False, feature_vec=False)
        #hog1 = get_hog_features(ch1, orient, pix_per_cell, cell_per_block, feature_vec=False)
        #hog2 = get_hog_features(ch2, orient, pix_per_cell, cell_per_block, feature_vec=False)
        #hog3 = get_hog_features(ch3, orient, pix_per_cell, cell_per_block, feature_vec=False)
    
    
    for xb in range(nxsteps):
        for yb in range(nysteps):
            ypos = yb*cells_per_step
            xpos = xb*cells_per_step
            # Extract HOG for this patch


            xleft = xpos*pix_per_cell
            ytop = ypos*pix_per_cell

            # Extract the image patch
            if hist_feat or spatial_feat:
                subimg = cv2.resize(ctrans_tosearch[ytop:ytop+window, xleft:xleft+window], (64,64))
          
            # Get color features
            file_features =[]
            if spatial_feat:
                spatial_features = bin_spatial(subimg, size=spatial_size)
                file_features.append(spatial_features)
            
            if hist_feat:
                hist_features = color_hist(subimg, nbins=hist_bins)
                file_features.append(hist_features)
            
            if hog_feat:
                if hog_channel == 'ALL':
                    hog_features_window = []
                    for channel in range(ctrans_tosearch.shape[2]):
                        hog_features_window.append(hog_features[channel][ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel())
                    hog_features_window = np.hstack(hog_features_window)        
                else:
                    hog_features_window = hog_features[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel()
                
                #hog_feat1 = hog1[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel() 
                #hog_feat2 = hog2[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel() 
                #hog_feat3 = hog3[ypos:ypos+nblocks_per_window, xpos:xpos+nblocks_per_window].ravel() 
                #hog_features = np.hstack((hog_feat1, hog_feat2, hog_feat3))
                file_features.append(hog_features_window)

            # Scale features and make a prediction
            test_features = X_scaler.transform(np.hstack(np.concatenate(file_features)).reshape(1, -1))    
            #test_features = X_scaler.transform(np.hstack((shape_feat, hist_feat)).reshape(1, -1))    
            test_prediction = svc.predict(test_features)
            
            if test_prediction == 1:
                xbox_left = np.int(xleft*scale)
                ytop_draw = np.int(ytop*scale)
                win_draw = np.int(window*scale)
                #cv2.rectangle(draw_img,(xbox_left, ytop_draw + ystart),(xbox_left + win_draw, ytop_draw + win_draw + ystart),(0,0,255),6) 
                output_windows.append(((xbox_left, ytop_draw + ystart),(xbox_left + win_draw, ytop_draw +win_draw + ystart)))

def train_classifier(image_path, 
                     color_space, 
                     spatial_size, 
                     hist_bins,
                     orient,
                     pix_per_cell,
                     cell_per_block,
                     hog_channel,
                     spatial_feat,
                     hist_feat,
                     hog_feat):
    """ Return a trained classifier and scaler"""
    # Read in cars and notcars
    img_pattern = '{}/*/*/*.png'.format(image_path)
    images = glob.glob(img_pattern)
    cars = []
    notcars = []
    for image in images:
        if 'non-' in image:
            notcars.append(image)
        else:
            cars.append(image)    
    print("Extracting features for non-car images")
    notcar_features = extract_features(notcars, color_space=color_space, 
                        spatial_size=spatial_size, hist_bins=hist_bins, 
                        orient=orient, pix_per_cell=pix_per_cell, 
                        cell_per_block=cell_per_block, 
                        hog_channel=hog_channel, spatial_feat=spatial_feat, 
                        hist_feat=hist_feat, hog_feat=hog_feat)  
    X = np.vstack((car_features, notcar_features)).astype(np.float64)   
    print("Extracting features for car images")
    car_features = extract_features(cars, color_space=color_space, 
                        spatial_size=spatial_size, hist_bins=hist_bins, 
                        orient=orient, pix_per_cell=pix_per_cell, 
                        cell_per_block=cell_per_block, 
                        hog_channel=hog_channel, spatial_feat=spatial_feat, 
                        hist_feat=hist_feat, hog_feat=hog_feat)
    # Fit a per-column scaler
    X_scaler = StandardScaler().fit(X)
    # Apply the scaler to X
    scaled_X = X_scaler.transform(X)  
    # Define the labels vector
    y = np.hstack((np.ones(len(car_features)), np.zeros(len(notcar_features))))
    
    # Split up data into randomized training and test sets
    rand_state = np.random.randint(0, 100)
    X_train, X_test, y_train, y_test = train_test_split(
        scaled_X, y, test_size=0.2, random_state=rand_state)
    
    print('Using:',orient,'orientations',pix_per_cell,
        'pixels per cell and', cell_per_block,'cells per block')
    print('Feature vector length:', len(X_train[0]))
    print ('Using {} examples for training and {} examples for testing'.format(X_train.shape[0], X_test.shape[0]))
    
    # Use a linear SVC 
    svc = SVC()#LinearSVC()
    # Check the training time for the SVC
    t=time.time()
    svc.fit(X_train, y_train)
    t2 = time.time()
    print(round(t2-t, 2), 'Seconds to train SVC...')
    # Check the score of the SVC
    print('Test Accuracy of SVC = ', round(svc.score(X_test, y_test), 4))
    # returning the trained classifier
    return svc, X_scaler

def get_classifier_scaler():
    """A wrapper around classifie trainer/retriever"""
    training_file = 'classifier.pickle'
    if os.path.exists(training_file):
        with open(training_file,'rb') as f:
            object = pickle.load(f)
            svc = object['classifier']
            X_scaler = object['scaler']
            #svc = object['classifier']
    else:
        svc,X_scaler = train_classifier('./train_set', 
                           color_space=color_space, 
                           orient=orient, 
                           pix_per_cell=pix_per_cell,
                           cell_per_block=cell_per_block, 
                           hog_channel=hog_channel, 
                           spatial_size=spatial_size,
                           hist_bins=hist_bins, 
                           spatial_feat=spatial_feat,
                           hist_feat=hist_feat,
                           hog_feat=hog_feat)
        with open(training_file,'wb') as f:
            object = {}
            object['classifier']=svc
            object['scaler']=X_scaler
            pickle.dump(object, f)
    return svc, X_scaler

svc, X_scaler = get_classifier_scaler()

def process_single_image(img, image_name, draw_image = True):
    """Find  cars in a single image in all scales using all parameters and classifier"""
    output_windows = []
    heatmap = np.zeros((img.shape[0],img.shape[1]))
    output_windows =[]
    threads = {}
    for count, scale in enumerate(scales):
        #threads[count] = threading.Thread(name="worker_scale_{}".format(scale),
        #                                      target=find_cars, 
        #                                      args=(img, 
        logging.debug("Starting vehicle detection in image <{}> in scale <{}>".format(image_name,scale))
        find_cars(  img,                                       
                    output_windows, 
                    ystart, 
                    ystop, 
                    scale, 
                    svc, 
                    X_scaler, 
                    orient, 
                    pix_per_cell, 
                    cell_per_block, 
                    hog_channel,
                    spatial_size, 
                    hist_bins,
                    spatial_feat,
                    hist_feat,
                    hog_feat
                    ) #)
        logging.debug("Ended vehicle detection in image <{}> in scale <{}>".format(image_name,scale))  
        #threads[count].start()     
    #for key in threads.keys():
    #        threads[key].join()
    #adding those detections on the heatmap
    add_heat(heatmap,output_windows)
    cv2.imwrite("temp/{}_original.jpg".format(image_name), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    new_img = draw_boxes(img, output_windows, color=(255, 255, 255), thick=6)
    new_img = cv2.cvtColor(new_img, cv2.COLOR_RGB2BGR)
    cv2.imwrite("temp/{}_individual_detections.jpg".format(image_name), new_img)
    fig, ax = plt.subplots(nrows=1, ncols =1)
    ax.imshow(heatmap/heatmap.max(), cmap='hot')
    fig.savefig('temp/{}_heatmap.jpg'.format(image_name))
    plt.close(fig)
    apply_threshold(heatmap, HEAT_THRESHOLD)
    cv2.imwrite("temp/{}_heatmap_thresholded.jpg".format(image_name), heatmap/heatmap.max()*255)
    labels = label(heatmap)
    cv2.imwrite("temp/{}_labeled_heatmap.jpg".format(image_name), labels[0]/labels[0].max()*255)
    if draw_image:
        draw_img = draw_labeled_bboxes(img, labels, draw_image)
        cv2.imwrite("temp/{}_final.jpg".format(image_name), cv2.cvtColor(draw_img, cv2.COLOR_RGB2BGR))
        return labels, draw_img
    else:
        return labels



class ImageJob(object):
    """Package for carrying jobs around producer, consumer, and recorder. Encapsulates all job values"""
    def __init__(self, img, image_name, type, filename = None, output_filename = None):
        self.img = img
        self.image_name = image_name
        self.type = type
        self.filename = filename
        self.output_filename = output_filename
        self.labels = None
        self.draw_img = None
        self.completed = False

class ImageDirJobsProducerThread(threading.Thread):
    """Class for producing image-jobs that are handled by both consumer and recorder"""
    def __init__(self, source_dir, dest_dir, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(ImageDirJobsProducerThread,self).__init__()
        self.target = target
        self.name = name
        self.source_dir = source_dir
        self.dest_dir = dest_dir
        self.queue  = Queue.Queue(BUF_SIZE)
        self.result_dict = {}

    def run(self):
        image_files = glob.glob("{}/*.jpg".format(self.source_dir))  
        while len(image_files)>0:
            if not self.queue.full():
                file = image_files[0]
                img = cv2.imread(file)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                output_file = "{}/{}".format(self.dest_dir, os.path.basename(file))
                job_name = os.path.basename(file)
                job = ImageJob(img = img, 
                               image_name = job_name, 
                               type = "file", 
                               filename = file, 
                               output_filename = output_file)
                self.queue.put(job)
                del image_files[0] #removing from the list of file to be processed
                time.sleep(1)
        return

class ImageDirJobsConsumerThread(threading.Thread):
    """Class for processing image jobs. It is assumed that there are a number of consumers running in parallel"""
    def __init__(self, producer, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(ImageDirJobsConsumerThread,self).__init__()
        self.target = target
        self.name = name
        self.producer = producer
        self.queue = producer.queue
        return

    def run(self):
        while not self.queue.empty():
            job = self.queue.get()
            labels, draw_img = process_single_image(job.img, job.image_name, draw_image = True)
            job.labels = labels
            job.draw_img = draw_img
            job.completed = True
            logging.debug("Found {} vehicles. Saving image under {}".format(labels[1], job.output_filename))
            draw_img = cv2.cvtColor(draw_img, cv2.COLOR_RGB2BGR)
            cv2.imwrite(job.output_filename, draw_img)
            time.sleep(1)
        return

def process_image_dir(source_dir, dest_dir): 
    """Meta-function to start producer and multiple consumers to process images in a directory"""
    "# getting a list of images from the source directory"
    image_files = glob.glob("{}/*.jpg".format(source_dir))   
    producer = ImageDirJobsProducerThread(source_dir = source_dir, dest_dir = dest_dir, name = "Producer")
    producer.start()
    
    num_threads = min(len(image_files), NUM_THREADS)
    consumers = []
    for i in range(num_threads):
        time.sleep(1)
        consumers.append(ImageDirJobsConsumerThread(producer, name = "Consumer #{}".format(i)))
        consumers[i].start()
    
    # waiting for threads to complete
    for i in range(num_threads):
        consumers[i].join()

class VideoJobsOpenCVProducerThread(threading.Thread):
    """Job producer from video based on OpenCV library"""
    def __init__(self, source_video_filename, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(VideoJobsOpenCVProducerThread,self).__init__()
        self.target = target
        self.name = name
        self.source_video_filename = source_video_filename
        self.done = False
        self.fps = 0
        self.width = 0
        self.height = 0
        self.cnt = 0
        self.total_frames = 0
        self.queue  = Queue.Queue(BUF_SIZE)
        self.result_dict = {}

    def run(self):
        input_clip = cv2.VideoCapture(self.source_video_filename)
        while not input_clip.isOpened():
            input_clip = cv2.VideoCapture(self.source_video_filename)
            time.sleep(1)
            logging.debug("Wait for the header")

        total_frames = input_clip.get(cv2.CV_CAP_PROP_FRAME_COUNT)

        while True:
            flag, frame = input_clip.read()
            if flag:
                # The frame is ready and already captured
                pos_frame = input_clip.get(cv2.CV_CAP_PROP_POS_FRAMES)
                frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                if not self.width:
                    self.height, self.width = frame.shape[0],frame.shape[1]
                while (q.full()):
                    time.sleep(1)
                job_name = "frame_{}".format(self.cnt), 
                job = ImageJob(img = frame, 
                               image_name = job_name, 
                               type = "video")
                q.put(job)
                self.cnt+=1
            else:
                # The next frame is not ready, so we try to read it again
                input_clip.set(cv2.CV_CAP_PROP_POS_FRAMES, pos_frame-1)
                time.sleep(1)
        
            if input_clip.get(cv2.CV_CAP_PROP_POS_FRAMES) == input_clip.get(cv2.CV_CAP_PROP_FRAME_COUNT):
                # If the number of captured frames is equal to the total number of frames,
                # we stop
                break
        #all frames have been queued
        self.done = True
        input_clip.release()
        logging.debug("Finished producing frames. Total num of frames = {}".format(self.cnt))
        return

class VideoJobsMoviePyProducerThread(threading.Thread):
    """Job producer from video based on MoviePy framework"""
    def __init__(self, source_video_filename, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(VideoJobsMoviePyProducerThread,self).__init__()
        self.target = target
        self.name = name
        self.source_video_filename = source_video_filename
        self.done = False
        self.fps = 0
        self.width = 0
        self.height = 0
        self.cnt = 0
        self.duration = 0
        self.queue  = Queue.Queue(BUF_SIZE)
        self.result_dict = {}

    def run(self):
        input_clip = VideoFileClip(self.source_video_filename)
        self.fps = input_clip.fps
        self.duration = input_clip.duration
        for frame in input_clip.iter_frames():
            if self.cnt > 7:
                break
            if not self.width:
                self.height, self.width = frame.shape[0],frame.shape[1]
            while (self.queue.full()):
                time.sleep(1)
            job_name = "frame_{}".format(self.cnt)
            job = ImageJob(img = frame, 
                           image_name = job_name, 
                           type = "video")
            self.queue.put(job)
            logging.debug("Enqueued {}".format(job_name))
            self.cnt+=1
            time.sleep(1)

        #all frames have been queued
        self.done = True
        logging.debug("Finished producing frames. Total num of frames = {}".format(self.cnt))
        return
        
    
class VideoJobsConsumerThread(threading.Thread):
    """Class of image consumer for jobs coming from video"""
    def __init__(self, video_producer,group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(VideoJobsConsumerThread,self).__init__()
        self.target = target
        self.name = name
        self.producer = video_producer
        self.queue = self.producer.queue
        self.result_dict = self.producer.result_dict

    def run(self):
        while not self.queue.empty():
            job = self.queue.get()
            labels = process_single_image(job.img, job.image_name, draw_image = False)
            frame_num = int(job.image_name.split("_")[1])
            job.labels = labels
            job.completed = True
            logging.debug("Found {} vehicles. Passing image under {} for video concatenation".format(labels[1], job.image_name))
            self.result_dict[frame_num] = job
            time.sleep(1)
        logging.debug("Finished processing frames")   
        return
    
class VideoJobsOpenCVRecorderThread(threading.Thread):
    """Class of video recording from jobs also coming from video. Based on OpenCV framework"""
    def __init__(self, dest_video_filename, video_producer,group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(VideoJobsOpenCVRecorderThread,self).__init__()
        self.target = target
        self.name = name
        self.dest_video_filename = dest_video_filename
        self.fps = video_producer.fps
        self.width = video_producer.width
        self.height = video_producer.height
        self.producer = video_producer
        self.curr_frame = 0
        self.result_dict = self.producer.result_dict
        self.prev_img = None # for tracking purposes we save the state of the previous images (except#0)
        self.prev_labels = None #for tracking purposes we save the state of the previous labels 

    def run(self):
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'x264')
        out = cv2.VideoWriter(self.dest_video_filename,fourcc, self.fps, (self.width,self.height))
        logging.debug("Num of results = {}".format(len(result_dict.keys())))
        logging.debug("Producer done? {}".format(self.producer.done))
        while not (len(result_dict.keys())==0 and self.producer.done):
            if self.curr_frame  in result_dict.keys():
                job = self.result_dict[self.curr_frame]
                del self.result_dict[self.curr_frame]
                if self.curr_frame ==0:
                    out.write(job.img)
                    self.prev_img = job.img
                    sellf.prev_labels = job.labels
                else:
                    # iterating through labeled components and supressing those of them 
                    # that do not intersect with the previoius image's heatmap, thus filtering spurious detections
                    temp = copy.deepcopy(job.labels) #making a backup for 
                    for car_number in range(1, temp[1]+1):
                        if np.logical_and((temp[0] == car_number), prev_labels[0]).sum()==0:
                            temp[0][temp[0] == car_number] = 0
            
                    self.prev_labels = job.labels
                    self.prev_img = copy.deepcopy(job.img)
                    draw_img = draw_labeled_bboxes(job.img, temp)
                    draw_img = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
                    out.write(job.draw_img)
                    logging.debug("Wrote {} to {}".format(job.image_name,self.dest_video_filename ))
            
                self.curr_frame+=1
            time.sleep(1)
                
        out.release()

        return
    
class VideoJobsMoviePyRecorderThread(threading.Thread):
    """Class of video recording from jobs also coming from video. Based on OpenCV framework"""
    def __init__(self, dest_video_filename, video_producer,group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(VideoJobsMoviePyRecorderThread,self).__init__()
        self.target = target
        self.name = name
        self.dest_video_filename = dest_video_filename
        self.fps = video_producer.fps
        self.width = video_producer.width
        self.height = video_producer.height
        self.producer = video_producer
        self.duration = (self.producer.duration*self.fps-2)/self.fps
        self.curr_frame = 0
        self.result_dict = self.producer.result_dict
        self.prev_img = None # for tracking purposes we save the state of the previous images (except#0)
        self.prev_labels = None #for tracking purposes we save the state of the previous labels 
        logging.debug("Generating video sequence of length = {} under fps = {}".format(self.duration,self.fps))

    def run(self):
        # Define the codec and create VideoWriter object
        output_clip = VideoClip(self.make_frame, duration = self.duration) 
        output_clip.write_videofile(self.dest_video_filename,  fps = self.fps)
        return
    
    def make_frame(self,t):
        while not self.curr_frame  in self.result_dict.keys():
            #logging.debug('Waiting 10 seconds to check for a frame')
            time.sleep(1)
        
        job = self.result_dict[self.curr_frame]
        del self.result_dict[self.curr_frame]
        output_filename = "temp/{}_final.jpg".format(job.image_name)
        if self.curr_frame == 0:
            draw_img = copy.deepcopy(job.img) # to return from this routine
            draw_img2 = cv2.cvtColor(job.img,cv2.COLOR_RGB2BGR) # to save as an image on disk    
            cv2.imwrite(output_filename,draw_img2)
            self.prev_img = job.img
            self.prev_labels =  job.labels
        else:
            # iterating through labeled components and supressing those of them 
            # that do not intersect with the previoius image's heatmap, thus filtering spurious detections
            temp = copy.deepcopy(job.labels) #making a backup for 
            for car_number in range(1, temp[1]+1):
                if np.logical_and((temp[0] == car_number), self.prev_labels[0]).sum()==0:
                    temp[0][temp[0] == car_number] = 0
            
            self.prev_labels = job.labels
            self.prev_img = copy.deepcopy(job.img)
            draw_img = draw_labeled_bboxes(job.img, temp)
            draw_img2 = cv2.cvtColor(draw_img,cv2.COLOR_RGB2BGR)
            cv2.imwrite(output_filename,draw_img2)
            cv2.imwrite("temp/{}_label_filtered.jpg".format(job.image_name),temp[0]/temp[0].max()*255)
        self.curr_frame+=1
        logging.debug("Wrote {} to {}".format(job.image_name,self.dest_video_filename ))
        return draw_img
    
    
def process_video(source_filename, dest_filename):
    """Meta function/wrapper that launches video processing by callling on producer, consumers, and recorder"""
    producer = VideoJobsMoviePyProducerThread(source_filename, name = "Producer")
    producer.start()

    consumers = []
    for i in range(NUM_THREADS):
        time.sleep(1)
        consumers.append(VideoJobsConsumerThread(producer, name = "Consumer #{}".format(i)))
        consumers[i].start()
    
    recorder = VideoJobsMoviePyRecorderThread(dest_filename, producer, name="Recorder")
    recorder.start()
    # waiting for threads to complete
    for i in range(NUM_THREADS):
        consumers[i].join()
    recorder.join()
    producer.join()

def process_image(filename): 
    """Meta-function process a single image and display results"""
    "# getting a list of images from the source directory"
    img = cv2.imread(filename)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    labels, draw_img = process_single_image(img, os.path.basename(filename).split('.')[0], draw_image=True)
    #plt.imshow(draw_img)
    
#process_image_dir("test_images", "output_images")
process_video('/home/sfefilatyev/projects/python_projects/CarND-Vehicle-Detection/test_video.mp4', 'test_output.mp4')
#process_video('project_video.mp4', 'project_video_output.mp4')
#process_image('test_images/test1.jpg')
#img_files = ['test_images/test1.jpg','test_images/test2.jpg','test_images/test3.jpg','test_images/test4.jpg','test_images/test5.jpg','test_images/test6.jpg']


