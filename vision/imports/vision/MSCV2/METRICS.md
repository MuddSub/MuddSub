# Tensor Board Metrics
This file contains brief explanations for each of the metrics that are graphed in 
tensor board during training. Note that all statistics are calculated for _each_ detection layer, and averaged over the number of detection layers layers _and_ the number of batches. So when you look at (say) loss, you're really seeing the average loss per detection layer per batch.

## Class Accuracy
This plots class accuracy, which is calculated as being the number of correct class predictions for each cell/anchor box that was responsible for finding an object in that cell, divided by the total number of objects that should have been detected.

## Confidence
There are two lines for this plot: one is the average object confidence value of the network in cells/anchor boxes responsible for predicting an object, and the second is the average object confidence value where there was no object present.

## Loss
This is just the average loss per detection layer per batch.

## Loss Breakdown
The breakdown of (scaled) losses for the network. Lines include bbox-loss (sum of x-loss, y-loss, w-loss, h-loss), conf-loss (sum of obj-conf-loss), and cls-loss.

## Note: Detections
For the following metrics, an object is classified as "detected" if the network had a confidence value > 0.5 and correctly predicted the class.

## Percent Correct Detections (aka Recall)
There are two lines that are tracking similar things. They are both calculated by taking all the objects that were detected and asking what percent of them had a bounding box IoU > some threshold. For the first line (recall50) this threshold is 0.5. for the second (recall75) it's 0.75.

## Precision
Precision is calculated as the number of objects detected with a bounding box IoU > 0.5 divided by the number of detections the network gave a confidence of > 0.5 to.

## Test Loss
This is just the loss of the network on the test set of data. This helps to give a good idea of if/when the model starts over fitting to the training data.