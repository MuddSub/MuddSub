''' detector.py
this file contains the Detector class, which kind of works as a wrapper to a
pre-trained and saved yolo network. it can do things like make detections on
images, save copies of the images with the detections shown, etc.
'''
import os
import torch
import cv2
from vision.YOLO.util import unique, c1c2_iou_mat
from vision.YOLO.datamanager.datamanager import DataManager


class Detector(object):
    ''' Detector class
    this class takes in a yolo network and uses it to make detections on images.

    --- args ---
    yolo_net_path : str
        the path to the yolo network to use to make detections.
    CUDA : bool, optional (default=False)
        if true, detections will use the GPU instead of the CPU (default).
    '''
    def __init__(self, yolo_net_path, CUDA=False):
        # +++ set device variable
        self.device = 'cuda:0' if CUDA else 'cpu'

        # +++ load and configure the yolo network
        f = open(yolo_net_path, 'rb') 
        self.model = torch.load(f)
        f.close()
        self.model.to(self.device)
        self.model.eval()

        # +++ save important attributes
        self.input_dim = self.model.input_dim
        self.num_classes = self.model.num_classes

    # +++ internal methods
    def _NMS_prediction(self, predictions, nms_thresh):
        ''' +++ INTERNAL FUNCTION +++ 
        takes in a set of predictions and applies non-max suppression to weed
        out duplicate predictions.

        --- args ---
        predictions : torch.FloatTensor with size (_, 7)
            the predictions as output by the network, the first dimension should
            contain [xmin, xmax, ymin, ymax, conf, cls conf, cls].
        
        --- returns ---
        torch.FloatTensor with size (_, 7) : the refined predicticions
        '''
        # +++ first, we sort the predictions by confidence for good NMS
        conf_sorted_idxs = torch.sort(
            predictions[:,4], descending=True)[1]
        predictions = predictions[conf_sorted_idxs] # sort in place

        # +++ loop for NMS
        i = 0 # index counter
        while i < predictions.size(0):
            # get the ious of this box with all the boxes after it
            ious = c1c2_iou_mat(
                predictions[i,:4].unsqueeze(0), 
                predictions[i+1:,:4])[0]
            
            # get the indexes for detections with IoU < nms_thresh
            iou_idxs = torch.where(ious < nms_thresh)[0]

            iou_idxs = iou_idxs + i+1 # add i+1 to get absolute indecies of these

            # eliminate the predictions that have ious > nms_thresh
            predictions = torch.cat((
                predictions[:i+1], # all predictions up to and including this one
                predictions[iou_idxs] # all other predictions with iou < thresh
            ), dim=0)

            i += 1 # increment index
        
        return predictions

    # +++ methods
    def get_detections(self, x, targets=None, conf_thresh=0.5, nms_thresh=0.3, nms_by='class'):
        ''' get detections method
        this method gets the network's detections on a set of input data. it 
        uses a confidence threshold and non-max supression to weed out low-
        confidence or duplicate predictions.
        
        --- args ---
        x : torch.FloatTensor
            the input tensor for the network, should have size 
            (batch_size, 3, input_dim, input_dim).
        targets : torch.FloatTensor, optional (defualt=None)
            the targets bounding boxes that should be detected.
        conf_thresh : float, optional (defualt=0.5)
            confidence threshold - detections with a confidence value below this
            threshold will be ignored.
        nms_thresh : float, optional (defualt=0.3)
            detections that fall under the same "category" (as defined by nms_by
            variable) with an IOU above this thresh will be collapsed to the one
            with a greater confidence.
        nms_by : string, optional (defualt='class')
            'class' - for classwise NMS
            'image' - for NMS independent of class
            'none' - for no NMS being applied at all
        
        --- returns ---
        torch.FloatTensor : the predicitons that the network is making, in the
            shape of a tensor with size (num_detections, 8) like so:
            [...
             0          1     2     3     4     5         6           7
            [image num, xmin, ymin, xmax, ymax, box conf, class conf, class num]
            ...]
        float OR None : loss of the network on the given batch of images. if no
            targets were given, this will be None.
        '''
        # +++ get ready
        x = x.to(self.device)
        targets = targets.to(self.device) if targets != None else None

        # +++ make the prediction using the model
        with torch.no_grad():
            if targets != None:
                predictions, loss = self.model(x, targets=targets)
            else:
                predictions = self.model(x)
                loss = None
        
        # +++ first, narrow it down to only predictions with a confidence value
            # above the given threshold
        conf_mask = (predictions[:,:,4] > conf_thresh).unsqueeze(2)
        predictions = predictions * conf_mask

        # +++ make sure there are still predictions made
        if torch.nonzero(predictions, as_tuple=False).numel() == 0:
            return torch.FloatTensor(0,8).to(self.device), loss

        # +++ convert the output of the network which has the form
            # (centerx, centery, width, height, ...) into the more managable
            # form of (x_min, x_max, y_min, y_max)
        x_center = predictions[...,0].clone()
        y_center = predictions[...,1].clone()
        width = predictions[...,2].clone()
        height = predictions[...,3].clone()
        
        predictions[...,0] = x_center - width/2
        predictions[...,1] = y_center - height/2
        predictions[...,2] = x_center + width/2
        predictions[...,3] = y_center + height/2

        # +++ replace the one-hot encoding of classes with the class confidence
            # and class prediction
        cls_pred_conf, cls_pred = torch.max(predictions[...,5:], dim=2)
        predictions = torch.cat((
            predictions[...,:5], 
            cls_pred_conf.unsqueeze(2), 
            cls_pred.unsqueeze(2)), dim=2)

        # +++ now we go through the images one by one to get predicted classes
            # and do NMS
        batch_size = predictions.size(0)
        detections = torch.FloatTensor(0,8).to(self.device)
        nms_by = nms_by.lower()

        for img_i in range(batch_size):
            image_pred = predictions[img_i]

            # +++ eliminate all predictions that have been zerod out
            nonzero_mask = (image_pred[:,4] != 0)
            image_pred = image_pred[nonzero_mask]

            # +++ do NMS in the way perscribed
            if nms_by == 'class':
                img_classes = unique(image_pred[:,-1]) # all classes in image
                for cls_n in img_classes:
                    # get predictions of this class
                    image_pred_cls = image_pred[image_pred[:,-1] == cls_n]
                    
                    # do NMS on only that class
                    image_pred_cls = self._NMS_prediction(image_pred_cls, nms_thresh)
                    
                    # set the image index for these predictions
                    img_i_col = torch.FloatTensor(
                        image_pred_cls.size(0), 1).fill_(img_i).to(self.device)
                    image_pred_cls = torch.cat((img_i_col, image_pred_cls), dim=1)

                    # write output
                    detections = torch.cat((detections, image_pred_cls), dim=0)
            elif nms_by == 'image':
                # just do NMS on all the predictions in the image
                image_pred = self._NMS_prediction(image_pred, nms_thresh)

                # add the image index column
                img_i_col = torch.FloatTensor(
                    image_pred.size(0), 1).fill_(img_i).to(self.device)
                image_pred = torch.cat((img_i_col, image_pred), dim=1)

                # write output
                detections = torch.cat((detections, image_pred), dim=0)
            elif nms_by == 'none':
                # add the image index column
                img_i_col = torch.FloatTensor(
                    image_pred.size(0), 1).fill_(img_i).to(self.device)
                image_pred = torch.cat((img_i_col, image_pred), dim=1)

                # and then write the output
                detections = torch.cat((detections, image_pred), dim=0)
        
        # +++ return the detections and loss
        return detections, loss

    def detect(self, data_path, batch_size=1, conf_thresh=0.5, nms_thresh=0.3, 
    nms_by='class', colorfile='colors.pt', results_dir='results'):
        ''' detect method
        this method performs detection on images and writes the output as images
        with boxes around objects, where each box is labeled with (in order) the
        class prediction, the confidence that there's an object in the box, and
        the confidence that the class prediction is correct.

        --- args ---
        data_path : string
            the path to the data to detect, this should be a .pt file.
        batch_size : int, optional (defualt=1)
            the batch_size that data should be processed in. if it is 1 a loss
            score for each image will be attached in the top left corner.
        conf_thresh : float, optional (defualt=0.5)
            confidence threshold - detections with a confidence value below this
            threshold will be ignored.
        nms_thresh : float, optional (defualt=0.3)
            detections that fall under the same "category" (as defined by nms_by
            variable) with an IOU above this thresh will be collapsed to the one
            with a greater confidence.
        nms_by : string, optional (defualt='class')
            'class' - for classwise NMS
            'image' - for NMS independent of class
            'none' - for no NMS being applied at all
        colorfile : string, optional (default='colors.pt')
            the file that color data will come from.
        results_dir : string, optional (default='results')
            the directory where the results of detection will be saved.
            should colors be shuffled before detection.
        
        --- returns ---
        None
        '''
        # +++ load up the data using the datamanager
        data = DataManager(data_path)
        # get the batches from the datamanager *in order*
        batches = data.batches(batch_size=batch_size, shuffle=False, augmented=False)

        # +++ setup for loop to get detections
        all_detections = torch.FloatTensor(0,8).cpu()
        batch_losses = []

        # +++ loop to get the detections
        for batch_i, (x, y) in enumerate(batches):
            # get detections for the batch
            detections, loss = self.get_detections(x, targets=y, 
                conf_thresh=conf_thresh, nms_thresh=nms_thresh, nms_by=nms_by)
            batch_losses.append(loss.cpu().item())
            detections = detections.cpu()
            # add to the image index the number of images that came before this one
            detections[:, 0] += batch_size * batch_i
            # and now we can write the output
            all_detections = torch.cat((all_detections, detections), dim=0)

        # +++ use the datamanager to scale the detections onto the orig images
        detections = data.scale_detections(all_detections)

        # +++ make the set of loss labels (if applicable)
        if batch_size == 1:
            loss_lbls = [f'loss = {l:.5f}' for l in batch_losses]
        else:
            loss_lbls = None

        # +++ write the detections to the images
        det_images = data.write_boxes(detections, img_labels=loss_lbls, 
            colorfile=colorfile)

        # +++ make the results directory if it is't around
        results_dir = os.path.realpath(results_dir) # get the real path
        if not os.path.exists(results_dir): # if it doesn't exist
            os.mkdir(results_dir) # make it

        # +++ get the path for each detection image
        det_paths = [os.path.join(results_dir, f'det_{i:0>3}.jpg') \
            for i in range(data.get_len(False))] # list for that

        # +++ save the images !!!
        for path, img in zip(det_paths, det_images):
            cv2.imwrite(path, img)

if __name__ == '__main__':
    det = Detector('yv3m-t-416-0.pt', CUDA=True)

    import time

    tstart = time.time()
    det.detect('data/test-data-t-416.pt', batch_size=1,
        results_dir='results-yv3m-t-416-0', nms_by='image')
    tend = time.time()

    print(f'detection took {tend-tstart}')
