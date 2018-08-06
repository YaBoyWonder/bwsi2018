#!/usr/bin/env python

if(__name__ != "__main__"):
    raise Exception(__file__ + " can only be run as __main__")

#extracts all faces from a directory of pictures to form training set
import cv2
import argparse, os, math

parser = argparse.ArgumentParser(description="a tool for extracting faces from a directory of images (.png, .jpg only)")

parser.add_argument("-i", "--input-folder", required=True, dest="input_folder", type=str, help="the path to input images (only .jpg or .png images are used)")
parser.add_argument("-d", "--dump", required=True, dest="dump", type=str, help="directory to dump all the extracted faces")
parser.add_argument("-f", "--file-prefix", required=True, dest="file_prefix", type=str, help="file prefix for all extracted images")
args = parser.parse_args()

assert args.input_folder != "", "path to input images must not be an empty string"
assert args.dump != "", "path to dump directory must not be an empty string"

INPUT_FOLDER = args.input_folder
DUMP_PATH = args.dump
FILE_PREFIX = args.file_prefix

CASCADE_CLASSIFIER_PATH = "./haarcascade_frontalface_alt.xml"


haar_face_cascade = cv2.CascadeClassifier(CASCADE_CLASSIFIER_PATH)

def main():
    nface = 0
    nframe = 1

    input_image_names = os.listdir(INPUT_FOLDER)
    input_image_names = list(filter(lambda f: (".jpg" in f) or (".png" in f), input_image_names))

    NUMBER_PADDING = math.ceil(math.log10(len(input_image_names)))

    for input_image in input_image_names:
        img_path = os.path.join(INPUT_FOLDER, input_image)

        frame = cv2.imread(img_path)

        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = haar_face_cascade.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=5)

        for (x,y,w,h) in faces:

            print("face %d found in frame %d" % (nface, nframe))

            subframe = frame[y:y+h, x:x+w]

            #cv2.imshow("face", subframe)

            #if face found
            cv2.imwrite(os.path.join(DUMP_PATH , FILE_PREFIX + ('%%0%dd.png' % NUMBER_PADDING) % nface), subframe)
            nface += 1

            if(cv2.waitKey(1) & 0xFF == ord('q')):
                #cv2.destroyAllWindows()
                raise KeyboardInterrupt:
                nframe += 1
try:
    main()
except KeyboardInterrupt:
    exit()
