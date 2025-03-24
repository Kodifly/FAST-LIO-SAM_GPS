import cv2
import numpy as np

def simulate_hdr(image):
    """
    Simulate HDR effects on a single image using local tone mapping.
    """
    # Convert the image to float32 for processing
    image_float = image.astype(np.float32) / 255.0

    # Apply a tonemap algorithm (e.g., Drago Tonemap)
    tonemap_drago = cv2.createTonemapDrago(gamma=2.2, saturation=1.0, bias=0.85)
    # tonemap_manriuk = cv2.createTonemapMantiuk()
    # tonemap_reinhard = cv2.createTonemapReinhard()
    hdr_image = tonemap_drago.process(image_float)
    # hdr_image = tonemap_manriuk.process(image_float)
    # hdr_image = tonemap_reinhard.process(image_float)

    # Convert the result back to 8-bit
    hdr_image_8bit = np.clip(hdr_image * 255, 0, 255).astype(np.uint8)
    return hdr_image_8bit

# Example usage
input_image_path = "/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye/undistorted_image_5_params/2323.290820760.jpg"
output_image_path = "/home/kodifly/datasets/rosbag/sample_data/4km_50kmh_ouster_fisheye/HDR_image/2323.290820760.jpg"

# Read the input image
image = cv2.imread(input_image_path)
if image is None:
    print(f"Error: Unable to load image {input_image_path}.")
else:
    # Simulate HDR
    hdr_image = simulate_hdr(image)

    # Save the simulated HDR image
    cv2.imwrite(output_image_path, hdr_image)
    print(f"Simulated HDR image saved to {output_image_path}")