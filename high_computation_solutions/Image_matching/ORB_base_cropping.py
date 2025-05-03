import cv2
import matplotlib.pyplot as plt
import numpy as np
import os


def ORB_image_matching(img1, img2):
    img1 = cv2.imread(img1, cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread(img2, cv2.IMREAD_GRAYSCALE)

    orb = cv2.ORB_create()

    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    matches = bf.knnMatch(des1, des2, k=2)

    good_matches = [m for m, n in matches if m.distance < 0.65 * n.distance]

    points1 = np.float32([kp1[m.queryIdx].pt for m in good_matches])
    points2 = np.float32([kp2[m.trainIdx].pt for m in good_matches])

    img_matches = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None, flags=2)
    plt.imshow(img_matches), plt.show()
    x, y = np.mean(points2, axis=0)

    return int(x), int(y)


def main():
    print("Starting image processing")
    images = os.listdir('../seq_images/seq_ball')
    images = [os.path.join('../seq_images/seq_ball', img) for img in images]

    if not os.path.exists('../seq_images/cropped_seq'):
        os.mkdir('../seq_images/cropped_seq')

    for i in range(len(images) - 1):
        x, y = ORB_image_matching(images[i], images[i + 1])

        img = cv2.imread(images[i], cv2.COLOR_BGR2RGB)
        h, w = img.shape[:2]
        x1, x2 = max(0, int(x - w/3)), min(w, int(x + w/3))
        y1, y2 = max(0, int(y - h/2)), min(h, int(y + h/2))

        cropped_image = img[y1:y2, x1:x2]

        plt.imshow(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))  # Convert BGR to RGB for display
        plt.show()

        filename = os.path.join('../seq_images/cropped_seq', f'cropped{i}.jpg')

        cv2.imwrite(filename, cropped_image)

if __name__ == '__main__':
    main()
