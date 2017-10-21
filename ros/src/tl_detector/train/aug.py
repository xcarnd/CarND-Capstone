import cv2
import numpy as np
import random
import glob
import os
import tqdm

ORIG_INPUT = "train/"
AUG_OUTPUT = "aug/"

INPUT_SIZE = (600, 600)
TARGET_SIZE = (224, 224)

def augment(path):
    p = path.replace(ORIG_INPUT, "")

    d, b = os.path.dirname(p), os.path.basename(p)
    
    img = cv2.imread(path)
    rows, cols, _ = img.shape
    # let's perform the augmentation!
    # including:
    #
    # 1. the original sample, resized to (224, 224)
    resized = cv2.resize(img, TARGET_SIZE)
    cv2.imwrite(os.path.join(AUG_OUTPUT + d, b), resized)
    #
    # 2. randomly shifting 1
    x_shift = random.randint(-int(0.3 * INPUT_SIZE[0]), int(0.3 * INPUT_SIZE[0]))
    y_shift = random.randint(-int(0.3 * INPUT_SIZE[1]), int(0.3 * INPUT_SIZE[1]))
    M = np.float32([[1, 0, x_shift], [0, 1, y_shift]])
    translated = cv2.warpAffine(img, M, (cols, rows))
    translated = cv2.resize(translated, TARGET_SIZE)
    cv2.imwrite(os.path.join(AUG_OUTPUT + d,  "translated_" + b), translated)
    # 
    # 3. randomly cropped to (500, 500) the resized to (224,
    # 244). this will give a shifted less zoomed out version
    x_coord = random.randint(5, 100)
    y_coord = random.randint(5, 100)
    cropped_500 = img[y_coord:y_coord+500, x_coord:x_coord+500]
    cropped_500 = cv2.resize(cropped_500, TARGET_SIZE)
    cv2.imwrite(os.path.join(AUG_OUTPUT + d,  "cropped_500_" + b), cropped_500)
    # 
    # 4. like what 3 does but first randomly cropped to (400, 400)
    x_coord = random.randint(5, 200)
    y_coord = random.randint(5, 200)
    cropped_400 = img[y_coord:y_coord+400, x_coord:x_coord+400]
    cropped_400 = cv2.resize(cropped_400, TARGET_SIZE)
    cv2.imwrite(os.path.join(AUG_OUTPUT + d,  "cropped_400_" + b), cropped_400)
    
    # flip
    cv2.imwrite(os.path.join(AUG_OUTPUT + d,  "flipped_" + b), cv2.flip(resized, 1))
    cv2.imwrite(os.path.join(AUG_OUTPUT + d,  "flipped_translated_" + b), cv2.flip(translated, 1))
    cv2.imwrite(os.path.join(AUG_OUTPUT + d,  "flipped_cropped_500_" + b), cv2.flip(cropped_500, 1))
    cv2.imwrite(os.path.join(AUG_OUTPUT + d,  "flipped_cropped_400_" + b), cv2.flip(cropped_400, 1))    

if __name__ == '__main__':
    if not os.path.exists(AUG_OUTPUT):
        os.makedirs(AUG_OUTPUT + "green")
        os.makedirs(AUG_OUTPUT + "yellow")
        os.makedirs(AUG_OUTPUT + "red")
    for path in tqdm.tqdm(glob.glob(ORIG_INPUT + "**/*.jpg")):
        augment(path)
