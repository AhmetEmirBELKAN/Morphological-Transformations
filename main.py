import cv2
import numpy as np


class trackbar:
    def __init__(self):
        self.ilowH = 0
        self.ihighH = 179
        self.thresh=0
        self.thresh_max=255
        self.thresh_min=0
        self.ilowS = 0
        self.ihighS = 255
        self.ilowV = 0
        self.ihighV = 255
        self.erode=0
        self.dilate=0
        self.MORPH_OPEN=0
        self.MORPH_CLOSE=0
        self.kernel_size=5
        self.kernel = np.ones((self.kernel_size,self.kernel_size), np.uint8)
        self.Create_Trackbar()
        self.cap=cv2.imread("anjiomyolipom-resim.jpg")


    def Create_Trackbar(self):
        cv2.namedWindow('image')
        # cv2.createTrackbar('lowH','image',self.ilowH,179,self.callback)
        # cv2.createTrackbar('highH','image',self.ihighH,179,self.callback)
        # cv2.createTrackbar('lowS','image',self.ilowS,255,self.callback)
        # cv2.createTrackbar('highS','image',self.ihighS,255,self.callback)
        # cv2.createTrackbar('lowV','image',self.ilowV,255,self.callback)
        # cv2.createTrackbar('highV','image',self.ihighV,255,self.callback)
        cv2.createTrackbar('thresh_min','image',self.thresh_min,255,self.callback)
        cv2.createTrackbar('thresh_max','image',self.thresh_max,255,self.callback)
        cv2.createTrackbar('kernel_size','image',self.kernel_size,20,self.callback)
        cv2.createTrackbar('erode','image',self.erode,10,self.callback)
        cv2.createTrackbar('dilate','image',self.dilate,10,self.callback)
        cv2.createTrackbar('MORPH_OPEN','image',self.MORPH_OPEN,10,self.callback)
        cv2.createTrackbar('MORPH_CLOSE','image',self.MORPH_CLOSE,10,self.callback)

    def getTrackbarPos(self):
        self.ilowH = cv2.getTrackbarPos('lowH', 'image')
        self.ihighH = cv2.getTrackbarPos('highH', 'image')
        self.kernel_size=cv2.getTrackbarPos('kernel_size','image')
        self.kernel=np.ones((self.kernel_size,self.kernel_size), np.uint8)
        self.thresh_min = cv2.getTrackbarPos('thresh_min', 'image')
        self.thresh_max = cv2.getTrackbarPos('thresh_max', 'image')
        self.ilowS = cv2.getTrackbarPos('lowS', 'image')
        self.ihighS = cv2.getTrackbarPos('highS', 'image')
        self.ilowV = cv2.getTrackbarPos('lowV', 'image')
        self.ihighV = cv2.getTrackbarPos('highV', 'image')
        self.erode= cv2.getTrackbarPos('erode', 'image')
        self.dilate = cv2.getTrackbarPos('dilate', 'image')
        self.MORPH_OPEN= cv2.getTrackbarPos('MORPH_OPEN', 'image')
        self.MORPH_CLOSE= cv2.getTrackbarPos('MORPH_CLOSE', 'image')

    def callback(self,x):
        pass

    def morphologicaloperations(self):
            self.cap=cv2.imread("anjiomyolipom-resim.jpg")
            hsv = cv2.cvtColor(self.cap, cv2.COLOR_BGR2HSV)
            

            # lower_color = np.array([self.ilowH, self.ilowS,self.ilowV]) 
            # upper_color = np.array([self.ihighH, self.ihighS, self.ihighV]) 
            # hsv = cv2.inRange(hsv, lower_color, upper_color)

            
            opening = cv2.morphologyEx(hsv, cv2.MORPH_OPEN, self.kernel)
            closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, self.kernel)
            dilation = cv2.dilate(closing, self.kernel, iterations=self.dilate)
            erosion = cv2.erode(dilation, self.kernel, iterations=self.erode)
            img_blur = cv2.GaussianBlur(erosion, (7,7), 0)
            edges = cv2.Canny(img_blur,self.thresh_min , self.thresh_max)
            edges = cv2.dilate(edges, self.kernel, iterations=1)
            cap = cv2.bitwise_and(self.cap, self.cap, mask=edges)

            return cap,edges
    
    def FindContours(self,cap,edges):
        
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i, contour in enumerate(contours):

            next_contour = hierarchy[0][i][0]
            if next_contour != -1:
                next_parent_contour = hierarchy[0][next_contour][3]
                parent_contour = hierarchy[0][i][3]
                if parent_contour != -1 and hierarchy[0][parent_contour][3] == -1:
                    area = cv2.contourArea(contour)
                    print(f"area : {area}")  
                    if area>500:

                        cv2.drawContours(cap, contours, i, (0, 255, 0), 2)

    def Main(self):
        
        while True:
            self.getTrackbarPos()
            cap,edges=self.morphologicaloperations()
            self.FindContours(cap,edges)
            cv2.imshow('image', cap)
            cv2.waitKey(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                self.qcap.release()
                break
        cv2.destroyAllWindows()
        self.cap.release()


if __name__ == '__main__':
    tb=trackbar()
    tb.Main()