
import cv2 as cv
import numpy as np
from collections import deque

class ImageBuffer:

    def __init__(self, max_size) -> None:
        """
        @param max_size - maximum size of data structure
        """
        
        self.MAX_SIZE   = max_size
        self.img_queue  = deque()
        self.size       = len(self.img_queue) 

    def assignImg(self, img):
        # if queue is full then make space for update
        if (len(self.img_queue) >= self.MAX_SIZE):
            self.img_queue.popleft()    
        self.img_queue.append(img)
        
    def getItemCount(self):
        self.size = len(self.img_queue)
        return self.size

    def getImg(self):
        
        if (len(self.img_queue)):
            return True, self.img_queue.popleft()
        else:
            return False, None

    def displayMemoryUsed(self): pass
        # total_memory = 0
        # for img in self.img_queue:
