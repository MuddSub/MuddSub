import cv2
import matplotlib.pyplot as plt

def resize(image_path, save_path, new_height, new_width):
    # load the image
    
    img = cv2.imread(image_path)

    # plt.imshow(img)
    # plt.show()
    print("Original Dimension: ", img.shape)
    
    dim = (new_width, new_height)
    # resize the image
    resized = cv2.resize(img,dim, interpolation=cv2.INTER_AREA )
    # save the image 
    print("New Dimension: ", resized.shape)
    cv2.imwrite(save_path, resized)
   
    print(f"saved to {save_path}")


if __name__ == "__main__":
    print("hello world")
    image_path = "./../imports/vision/test_gate.jpg"
    save_path = "./../imports/vision/test_gate_resized.jpg"
    new_height = 480
    new_width = 640
    resize(image_path, save_path, new_height, new_width)