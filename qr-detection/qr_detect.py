import cv2
import matplotlib.pyplot as plt
import numpy as np

def scan_qr_with_matplotlib():
    cap = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()
    detected = False
    # result_text = ""

    if not cap.isOpened():
        print("❌ 카메라를 열 수 없습니다.")
        return

    plt.ion()  # 인터랙티브 모드 on
    fig, ax = plt.subplots()
    im = ax.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
    ax.axis("off")
    fig.canvas.manager.set_window_title("QR 실시간 스캐너 (ESC 닫기)")

    print("✅ QR 인식 시작. 정사각형 안에 QR 코드를 넣으세요. [ESC] 키로 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임 읽기 실패")
            break

        height, width, _ = frame.shape
        box_size = min(width, height) // 2
        top_left = ((width - box_size) // 2, (height - box_size) // 2)
        bottom_right = ((width + box_size) // 2, (height + box_size) // 2)

        # 가이드 박스 그리기
        cv2.rectangle(frame, top_left, bottom_right, (0, 255, 255), 2)

        # ROI 내부에서만 인식
        roi = frame[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0]]
        data, bbox, _ = detector.detectAndDecode(roi)

        if data and not detected:
            detected = True
            # result_text = data
            print(f"✅ QR 코드 인식됨: {data}")

        # if detected:
            # cv2.putText(frame, f"Detected: {result_text}", (30, 50),
                        # cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # BGR → RGB 변환 후 업데이트
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        im.set_data(rgb)
        fig.canvas.draw()
        fig.canvas.flush_events()

        # ESC 키 입력 확인
        if plt.waitforbuttonpress(0.001):  # True if key pressed
            print("🛑 키 입력 감지됨. 종료합니다.")
            break

    cap.release()
    plt.ioff()
    plt.close()

# 실행
if __name__ == "__main__":
    scan_qr_with_matplotlib()
