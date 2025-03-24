import cv2
import numpy as np
import os
from glob import glob

def adapt_exposure(image):
    """
    Adapt the exposure of an image using CLAHE and adaptive adjustments.
    """
    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_image)

    # Apply CLAHE to the V channel
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    v_clahe = clahe.apply(v)

    # Normalize the CLAHE V channel to [0, 1] for easier manipulation
    v_normalized = v_clahe.astype(np.float32) / 255.0

    # Define thresholds for dark and light regions
    dark_threshold = 0.2  # Pixels with brightness < 30% are considered dark
    light_threshold = 0.8  # Pixels with brightness > 70% are considered light

    # Adjust brightness: increase for dark regions, decrease for light regions
    v_adjusted = v_normalized.copy()
    # v_adjusted[v_normalized < dark_threshold] *= 1.4  # Brighten dark regions further
    # v_adjusted[v_normalized > light_threshold] *= 0.95  # Dim light regions slightly

    # Clamp values to [0, 1] to prevent overflow
    v_adjusted = np.clip(v_adjusted, 0, 1)

    # Scale back to [0, 255] and convert to uint8
    v_adjusted = (v_adjusted * 255).astype(np.uint8)

    # Merge the adjusted V channel back with H and S channels
    adjusted_hsv = cv2.merge([h, s, v_adjusted])

    # Convert back to BGR color space
    adjusted_image = cv2.cvtColor(adjusted_hsv, cv2.COLOR_HSV2BGR)
    return adjusted_image

def batch_process_images_in_sequence(input_folder, output_folder):
    """
    Process all images in the input folder in sequence, adapt exposure, and save corrected images.
    """
    # Create the output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Get all image files in the input folder and sort them alphabetically
    image_paths = sorted(
        glob(os.path.join(input_folder, "*.[jJ][pP][gG]")) + 
        glob(os.path.join(input_folder, "*.[pP][nN][gG]"))
    )

    # Optionally, sort by numeric part of filenames (if filenames contain numbers)
    image_paths = sorted(image_paths, key=lambda x: int(''.join(filter(str.isdigit, os.path.basename(x)))))

    # Process each image in sequence
    for i, image_path in enumerate(image_paths, start=1):
        # Load the image
        image = cv2.imread(image_path)
        if image is None:
            print(f"Error: Unable to load image {image_path}. Skipping...")
            continue

        # Adapt the exposure
        adjusted_image = adapt_exposure(image)

        # Save the adjusted image
        output_path = os.path.join(output_folder, os.path.basename(image_path))
        cv2.imwrite(output_path, adjusted_image)
        print(f"Processed image {i}/{len(image_paths)}: Saved to {output_path}")

# Example usage
input_folder = "/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye/undistorted_image_8_params"
output_folder = "/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye/CLAHE_image"

batch_process_images_in_sequence(input_folder, output_folder)