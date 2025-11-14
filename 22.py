import cv2
import numpy as np

image_names = ['1.jpg', '2.jpg', '3.jpg', '4.jpg']

for name in image_names:
    # ① 이미지 읽기
    img = cv2.imread(name)
    img = cv2.resize(img, (640, 480))
    
    # ② 노이즈 제거
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    
    # ③ BGR → HSV 변환
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # ④ 색상 범위 지정 (노란색 / 흰색)
    # ※ 조명 변화 대응을 위해 S, V 범위 좁게 설정
    lower_yellow = np.array([18, 80, 100])
    upper_yellow = np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_white = np.array([0, 0, 210])
    upper_white = np.array([180, 40, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    # ⑤ 노란색과 흰색 결합
    mask = cv2.bitwise_or(mask_yellow, mask_white)

    # ⑥ 형태학적 연산으로 잡음 제거 (모폴로지)
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)

    # ⑦ 윤곽선 검출
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = img.copy()
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 300:  # 너무 작은 점/노이즈 제외
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # ⑧ 최종 결과 표시 및 저장
    cv2.imshow(name, result)
    cv2.imwrite(f"result_{name}", result)

cv2.waitKey(0)
cv2.destroyAllWindows()
>>>>>>> first commit
