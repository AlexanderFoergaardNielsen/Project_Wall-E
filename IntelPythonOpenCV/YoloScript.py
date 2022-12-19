#!/usr/bin/env python3
import torch
import cv2
from PIL import Image
import numpy as np
import pathlib
from depthIntel import DepthCamera
#dc = DepthCamera('141222072962')
#gen_path = pathlib.Path.cwd()
#DEVICE = "cuda" \
#if torch.cuda.is_available() else "cpu"

#model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/alexander/PycharmProjects/yolov5/yolov5weights.pt')

class Yolo:

    def __init__(self):
        self.dc = DepthCamera('141222072962')#DepthCamera('827312070023')#DepthCamera('141222072962')
        self.gen_path = pathlib.Path.cwd()
        self.DEVICE = "cuda" \
        if torch.cuda.is_available() else "cpu"

        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/alexander/PycharmProjects/yolov5/yolov5weights.pt')

        #self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/alexander/PycharmProjects/yolov5/yolov5weights.pt')


    def yolov5(self, alig):
        litter_list=[0, 0, 0]
        ret, depth_frame, frame = self.dc.get_frame()
        frame = frame[:, :, [2, 1, 0]]
        frame = Image.fromarray(frame)
        frame = cv2.cvtColor(np.array(frame), cv2.COLOR_RGB2BGR)

        result = self.model(frame, size=640)
        cv2.imshow('YOLO', np.squeeze(result.render()))
        cv2.waitKey(1)

        result_array = result.xyxy[0].cpu().detach().numpy()
        if result_array.shape != (0, 6):
            print(result_array.shape)
            print('Confidence: ', result_array[0, 4])
            if result_array[0, 4] >= 0.8 and alig==False or result_array[0, 4] >= 0.6 and alig==True:
                print('Alignment Variable: ',alig)
                rows, columns = result_array.shape
                print('Rows: ', rows)
                dist_array = []
                # go through each element in arr
                row_loop = range(0, rows)
                for e in row_loop:
                    if result_array[e, 4] >= 0.8 and alig==False or result_array[e, 4] >= 0.6 and alig==True:
                        print('Element: ', e)
                        dist_array.append(result_array[e, 3])
                        print('Length Dist_array: ', len(dist_array))
                        e = +e

                    # if the element is higher than 42, set the value to True, otherwise False:
                if len(dist_array) > 0:
                    y_max = max(dist_array)
                    index_y_max = np.where(result_array == y_max)
                    row_y_max = index_y_max[0]
                    print('Max Y: ', y_max)
                    print('Row with biggest Y: ', row_y_max)
                    litter_count = len(dist_array)
                    print(f'Litter count in frame: {litter_count}')

                    x = (result_array[row_y_max, 0] + result_array[row_y_max, 2])
                    x = x / 2
                    x = int(x)
                    y = int(result_array[row_y_max, 3])
                    if y == 480:
                        y = 479
                    depth = depth_frame[y, x]
                    litter_list = (x, y, depth)


                    print(f'Litter List: {litter_list}')
                    print(f'Confidence: {result_array[row_y_max, 4]}')
        print(result_array)
        print(result_array.shape)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

        return litter_list

#yolo = Yolo()
#while True:
    #yolo.yolov5()
'''
while True:
    if sum(yolo.yolov5()) >0:
        (x, y, depth) = yolo.yolov5()
        print('Return: ',x, y, depth)
'''
'''
while True:
    ret, depth_frame, frame = dc.get_frame()
    frame = frame[:, :, [2,1,0]]
    frame = Image.fromarray(frame)
    frame = cv2.cvtColor(np.array(frame), cv2.COLOR_RGB2BGR)

    result = model(frame,size=640)
    cv2.imshow('YOLO', np.squeeze(result.render()))
    result_array = result.xyxy[0].cpu().detach().numpy()
    if result_array.shape != (0,6):
        print(result_array.shape)

        if result_array[0,4] >= 0.8:
            rows, columns = result_array.shape
            print('Rows: ',rows)
            dist_array =[]
            # go through each element in arr
            row_loop = range(0, rows)
            for e in row_loop:
                if result_array[e, 4] >= 0.8:
                    print('Element: ', e)
                    dist_array.append(result_array[e, 3])
                    print('Length Dist_array: ', len(dist_array))
                    e=+e

                # if the element is higher than 42, set the value to True, otherwise False:
            if len(dist_array) >0:
                y_max = max(dist_array)
                index_y_max = np.where(result_array==y_max)
                row_y_max = index_y_max[0]
                print('Max Y: ',y_max)
                print('Row with biggest Y: ',row_y_max)
                litter_count = len(dist_array)
                print(f'Litter count in frame: {litter_count}')

                x = (result_array[row_y_max,0] + result_array[row_y_max,2])
                x = x/2
                x = int(x)
                y = int(result_array[row_y_max,3])
                if y == 480:
                    y=479
                depth = depth_frame[y,x]
                litter_list = [x, y, depth]
                print(f'Litter List: {litter_list}')
                print(f'Confidence: {result_array[row_y_max,4]}')
    print(result_array)
    print(result_array.shape)
    #if  result.pandas().xyxy[0].iloc[0]['confidence'] != None:
        #print(result.pandas().xyxy[0].iloc[0]['confidence'])

    #if result.pandas().xyxy[0][4].numpy() >=0.8:
    #    print('Confidence Interval: ', result.pandas().xyxy[0][4].numpy())
    #result.pandas().xyxy
    result.print()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
'''


'''
import torch
import numpy as np
import cv2
import time
from IntelCamera import DepthCamera
dc = DepthCamera('141222072962')

class ObjectDetection:
    """
    Class implements Yolo5 model to make inferences on a youtube video using OpenCV.
    """

    def __init__(self):
        """
        Initializes the class with youtube url and output file.
        :param url: Has to be as youtube URL,on which prediction is made.
        :param out_file: A valid output file name.
        """
        self.model = self.load_model()
        self.classes = self.model.names
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("\n\nDevice Used:", self.device)

    def load_model(self):
        """
        Loads Yolo5 model from pytorch hub.
        :return: Trained Pytorch model.
        """
        model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/alexander/PycharmProjects/yolov5/yolov5weights.pt')

        return model

    def score_frame(self, frame):
        """
        Takes a single frame as input, and scores the frame using yolo5 model.
        :param frame: input frame in numpy/list/tuple format.
        :return: Labels and Coordinates of objects detected by model in the frame.
        """
        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)

        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
        return labels, cord

    def class_to_label(self, x):
        """
        For a given label value, return corresponding string label.
        :param x: numeric label
        :return: corresponding string label
        """
        return self.classes[int(x)]

    def plot_boxes(self, results, frame):
        """
        Takes a frame and its results as input, and plots the bounding boxes and label on to the frame.
        :param results: contains labels and coordinates predicted by model on the given frame.
        :param frame: Frame which has been scored.
        :return: Frame with bounding boxes and labels ploted on it.
        """
        labels, cord = results
        n = len(labels)
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(n):
            row = cord[i]
            if row[4] >= 0.2:
                x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(
                    row[3] * y_shape)
                bgr = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)

        return frame

    def __call__(self):
        """
        This function is called when class is executed, it runs the loop to read the video frame by frame,
        and write the output into a new file.
        :return: void
        """
        #cap = cv2.VideoCapture(0)

        while True:#cap.isOpened():

            ret, depth_frame, color_frame = dc.get_frame()
            start_time = time.perf_counter()
            #start_time = time.perf_counter()
            #ret, frame = cap.read()
            if not ret:
                break
            results = self.score_frame(color_frame)
            frame = self.plot_boxes(results, color_frame)
            end_time = time.perf_counter()
            fps = 1 / np.round(end_time - start_time, 3)
            cv2.putText(frame, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
            cv2.imshow("img", color_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


# Create a new object and execute.
detection = ObjectDetection()
detection()
'''