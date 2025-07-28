#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, PointCloud2, NavSatFix, Imu
import sensor_msgs.point_cloud2 as pc2
from datetime import datetime, timezone, timedelta


def save_bin(points_array, field_names, bin_filename):
	dtype = [(f, 'f4') for f in field_names]
	data = np.zeros(len(points_array), dtype=dtype)
	for f in field_names:
		data[f] = points_array[f]
	data.tofile(bin_filename)
	print(f"Saved point cloud as .bin: {bin_filename}")


def undistort_image(cv_image, camera_matrix, dist_coeffs):
	return cv2.undistort(cv_image, camera_matrix, dist_coeffs, None, camera_matrix)


def ts_to_str(sec: int, nsec: int, tz=timezone(timedelta(hours=8))) -> str:
	millis = nsec // 1_000_000
	base = datetime.fromtimestamp(sec, tz).strftime("%Y%m%d%H%M%S")
	return f"{base}{millis:03d}"


if __name__ == "__main__":
	 #  bags to read (add .bag suffix or full path)
	BAG_FILES = [
		"/home/kodifly/ssd1/2025-07-22-13-14-35.bag"
	]

	POINTCLOUD_TOPIC = "/ouster/points"
	GPS_TOPIC = "/gps/fix"
	IMU_TOPIC = "/ouster/imu"

	IMAGE_TOPICS = {
		"/DA5324655/image/compressed": "cam_DA5324655",
		"/DA5324645/image/compressed": "cam_DA5324645",
		"/DA5148683/image/compressed": "cam_DA5148683",
		"/DA4930148/image/compressed": "cam_DA4930148",
		"/DA6102933/image/compressed": "cam_DA6102933",
		"/DA5148680/image/compressed": "cam_DA5148680",
	}
	CAMERA_SERIES = {t: t.split("/")[2] for t in IMAGE_TOPICS}

	INTRINSICS = {
		"/DA5324655/image/compressed": (np.array([
			[1419.488223, 0, 2091.72389],
			[0, 1419.783836, 1231.71474],
			[0, 0, 1]]), np.array([0.4765817545, 0.0267510319, 1.495544387e-07, -2.581193063e-07,
									   8.833337814e-05, 0.8074516265, 0.1103335164, 0.001695886702])),
		"/DA5324645/image/compressed": (np.array([
			[1422.154509, 0, 2072.490584],
				[0, 1421.504888, 1216.098823],
				[0, 0, 1]
		]), np.array([0.5728396239, 0.04388080953, -3.660518528e-05, -2.037987016e-06, 0.0002235242205, 0.9067623525, 0.1550977017, 0.003418363523])),
		"/DA5148683/image/compressed": (np.array([
				[1418.225423, 0, 2046.160019],
				[0, 1417.934837, 1237.915652],
				[0, 0, 1]
			]), np.array([0.5705521112, 0.04216308511, -3.763745038e-05, 1.426525257e-05, 0.0001854266064, 0.9045889459, 0.1523478871, 0.003111423977])
		),
		"/DA4930148/image/compressed": (np.array([
			[1418.723501, 0, 2016.266075],
			[0, 1418.745041, 1237.536737],
			[0, 0, 1]
		]), np.array([0.6008030073, 0.05105767781, 1.91756943e-05, 3.502701816e-06, 0.0003079906229, 0.935219043, 0.1711876097, 0.00430346178])),
		"/DA5148680/image/compressed": (np.array([
			[1422.990622, 0, 2083.213929],
				[0, 1422.877393, 1229.036905],
				[0, 0, 1]
		]), np.array([0.5949228532, 0.04997329253, -5.025851349e-05, 5.032790752e-06, 0.0002994346306, 0.9288945431, 0.168177548, 0.004192957709])),
		"/DA6102933/image/compressed": (np.array([
				[1422.18129, 0, 2017.285053],
				[0, 1421.822766, 1232.094237],
				[0, 0, 1]
			]), np.array([0.602916894, 0.05030154934, -2.022050003e-05, 1.21141271e-05, 0.0002782603798, 0.9373883307, 0.1707547416, 0.00410583338]))
	}

	typemap = {POINTCLOUD_TOPIC: PointCloud2,
			   GPS_TOPIC:        NavSatFix,
			   IMU_TOPIC:        Imu,
			   **{t: CompressedImage for t in IMAGE_TOPICS}}

	bridge = CvBridge()

	for bag_path in BAG_FILES:
		# base_name = os.path.splitext(os.path.basename(bag_path))[0]
		base_name = "/home/kodifly/ssd1/13-14-35"

		PC_DIR = f"{base_name}/ouster_points"
		IMG_DIR = f"{base_name}/rectified_images"
		GPS_DIR = f"{base_name}"
		IMU_DIR = f"{base_name}"

		os.makedirs(PC_DIR,  exist_ok=True)
		os.makedirs(IMG_DIR, exist_ok=True)
		os.makedirs(GPS_DIR, exist_ok=True)
		os.makedirs(IMU_DIR, exist_ok=True)

		gps_tmp = os.path.join(GPS_DIR, "gps_data.txt")
		gps_f = open(gps_tmp, 'w')
		gps_f.write("#timestamp latitude longitude altitude\n")
		first_ts, last_ts = None, None

		imu_tmp = os.path.join(IMU_DIR, "imu_data.txt")
		imu_f = open(imu_tmp, 'w')
		imu_f.write("# timestamp orientation_x orientation_y orientation_z orientation_w " \
                 "angular_velocity_x angular_velocity_y angular_velocity_z " \
                 "linear_acceleration_x linear_acceleration_y linear_acceleration_z\n")
		first_ts_imu, last_ts_imu = None, None

		IMU_LOG_INTERVAL = 1000
		imu_count = 0

		print(f"Processing {bag_path} …")
		with rosbag.Bag(bag_path, "r") as bag:
			for topic, msg, _ in bag.read_messages(topics=list(typemap)):
				try:
					if topic == POINTCLOUD_TOPIC:
						fields = [f.name for f in msg.fields]
						pts = list(pc2.read_points(
							msg, skip_nans=True, field_names=fields))
						if not pts:
							continue
						arr = np.array(
							pts, dtype=[(f, np.float32) for f in fields])
						stamp = ts_to_str(
							msg.header.stamp.secs, msg.header.stamp.nsecs)
						save_bin(arr, fields, os.path.join(
							PC_DIR, f"lidar_{stamp}.bin"))
					elif topic == GPS_TOPIC:
						stamp = ts_to_str(msg.header.stamp.secs, msg.header.stamp.nsecs)
						if first_ts is None:
							first_ts = stamp
						last_ts = stamp
						gps_f.write(f"{stamp} {msg.latitude} {msg.longitude} {round(msg.altitude,3)}\n")
						# print(f"Processed gps messages ...")
					elif topic == IMU_TOPIC:
						stamp = ts_to_str(msg.header.stamp.secs, msg.header.stamp.nsecs)
						if first_ts_imu is None:
							first_ts_imu = stamp
						last_ts_imu = stamp

						line = f"{stamp} " \
                           f"{msg.orientation.x} {msg.orientation.y} {msg.orientation.z} {msg.orientation.w} " \
                           f"{msg.angular_velocity.x} {msg.angular_velocity.y} {msg.angular_velocity.z} " \
                           f"{msg.linear_acceleration.x} {msg.linear_acceleration.y} {msg.linear_acceleration.z}\n"

						imu_f.write(line)

						imu_count += 1
						if imu_count % IMU_LOG_INTERVAL == 0:
							print(f"Processed {imu_count} IMU messages ...")
					else:   # compressed image
						cv_img = bridge.compressed_imgmsg_to_cv2(
							msg, desired_encoding="bgr8")
						# K, D = INTRINSICS[topic]
						# rect = undistort_image(cv_img, K, D)
						rect = cv_img
						# h, w = rect.shape[:2]
						# if w > 1024:
						#    rect = cv2.resize(rect, (1024, int(h * 1024 / w)), interpolation=cv2.INTER_AREA)
						stamp = ts_to_str(
							msg.header.stamp.secs, msg.header.stamp.nsecs)
						out_folder = os.path.join(IMG_DIR, IMAGE_TOPICS[topic])
						os.makedirs(out_folder, exist_ok=True)
						fname = os.path.join(
							out_folder, f"cam_{CAMERA_SERIES[topic]}_{stamp}.jpg")
						cv2.imwrite(fname, rect)
						print(f"Saved undistorted image: {fname}")

				except Exception as e:
					print(f"Error on topic {topic}: {e}")
				# break   # comment these two breaks to process the whole bag
			# break
		gps_f.close()
		imu_f.close()
		if first_ts and last_ts:
			new_name = os.path.join(
				GPS_DIR, f"gps_data_{first_ts}_{last_ts}.txt")
			os.rename(gps_tmp, new_name)
			print(f"Saved GPS data: {new_name}")

		if first_ts_imu and last_ts_imu:
			new_name = os.path.join(IMU_DIR, f"imu_data_{first_ts_imu}_{last_ts_imu}.txt")
			os.rename(imu_tmp, new_name)
			print(f"Saved IMU data: {new_name}")
	print("Done. All bags processed.")
