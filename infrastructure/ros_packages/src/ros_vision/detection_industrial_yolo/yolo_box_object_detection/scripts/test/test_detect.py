from ultralytics import YOLO

# 加载模型
model = YOLO("../models/best.pt")

# 进行目标检测
results = model.predict(source=4, show=True, conf=0.4)

# 打印结果 (可选)
for result in results:
    print(result)