from ultralytics import YOLO
import cv2

# 加载模型
model = YOLO("../models/best.pt")

# 读取图像
image_path = 'test.jpg'
img = cv2.imread(image_path)

# 进行目标检测
results = model(img)

# 遍历检测结果并绘制边界框
for result in results:
    for bbox in result.boxes:
        # 获取边界框的坐标
        x1, y1, x2, y2 = map(int, bbox.xyxy[0])  # 将坐标转换为整数

        # 绘制矩形框
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 绿色边框，线宽为2

        # 获取中心点坐标和尺寸
        x_center, y_center, width, height = bbox.xywh[0]

        # 打印结果
        print("左上角坐标 (x1, y1):", x1, y1)
        print("右下角坐标 (x2, y2):", x2, y2)
        print("中心点坐标 (x_center, y_center):", x_center, y_center)
        print("宽度:", width)
        print("高度:", height)

# 缩放图像
scale_percent = 50  # 缩放比例，50%表示缩小一半
width = int(img.shape[1] * scale_percent / 100)
height = int(img.shape[0] * scale_percent / 100)
dim = (width, height)
resized_img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)

# 显示图像
cv2.imshow("Detected Image", resized_img)

try:
    # 使用无限循环保持窗口打开
    while True:
        if cv2.getWindowProperty("Detected Image", cv2.WND_PROP_VISIBLE) < 1:  # 检查窗口是否被关闭
            break
        cv2.waitKey(100)  # 等待100毫秒，避免CPU占用过高
except KeyboardInterrupt:
    # 捕获Ctrl+C中断
    print("程序已被中断，关闭窗口。")

# 关闭所有OpenCV窗口
cv2.destroyAllWindows()