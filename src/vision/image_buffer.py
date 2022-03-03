
import cv2 as cv
import numpy as np
from collections import deque

class ImageBuffer:

    def __init__(self, max_size) -> None:
        """ @param max_size - maximum size of data structure """
        
        self.MAX_SIZE   = max_size
        self.img_queue  = deque()
        self.item_count = 0 

    def pushImg(self, img):

        # return if invalid object is recieved
        if img == None: 
            return 

        img_copy = img.copy()

        # if queue is full then make space for update
        if (self.item_count  >= self.MAX_SIZE):
            _ = self.img_queue.popleft()    
        
        self.img_queue.append(img_copy)
        self.item_count = len(self.img_queue) 

    def popImg(self):
        
        if (self.item_count > 0):
            
            self.item_count = len(self.img_queue) 
            return (True, self.img_queue.popleft())

        else:
            return (False, None)
