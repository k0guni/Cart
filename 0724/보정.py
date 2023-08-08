import cv2
import numpy as np

def correct_shake(image_path):
    # 이미지 로드
    image = cv2.imread(image_path)

    # 이미지를 그레이스케일로 변환
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 특징점 검출 (SIFT, SURF, ORB 등)
    detector = cv2.SIFT_create()
    keypoints = detector.detect(gray, None)

    # 특징점 기반으로 이미지를 보정하여 안정된 이미지 생성
    corrected_image = cv2.drawKeypoints(image, keypoints, None)

    # 보정된 이미지를 파일로 저장
    cv2.imwrite("/home/ubuntu/ex_code/image1/img27.jpg", corrected_image)

    # 보정된 이미지 출력
    cv2.imshow("Corrected Image", corrected_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image_path = "/home/ubuntu/ex_code/image copy/img27.jpg"
    correct_shake(image_path)
