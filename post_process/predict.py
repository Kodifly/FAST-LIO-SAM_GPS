from ultralytics import YOLO

# Load the YOLOv11x model
model = YOLO('/home/kodifly/workspaces/data_collect_ws/src/mvs_ros_driver/models/best.pt')

# Only predict classes 0 and 1 (replace with correct indices for your model)
results = model.predict(
    source='/home/kodifly/ssd3/isds_pii/0515/rectified_image',
    save=True,
    project='/home/kodifly/ssd3/isds_pii/0515/yolo_results',
    name='predict_run',
    conf=0.01,
    classes=[0, 1]  # specify class indices here
)