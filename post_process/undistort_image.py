import cv2
import numpy as np
import os

def undistort_image(image_path, camera_matrix, dist_coeffs, output_path):
    """
    Undistorts an image using camera intrinsics and distortion coefficients.

    Parameters:
        image_path (str): Path to the input distorted image.
        camera_matrix (np.ndarray): 3x3 camera intrinsic matrix.
        dist_coeffs (np.ndarray): Distortion coefficients (5 or 8 parameters).
        output_path (str): Path to save the undistorted image.
    """
    # Load the distorted image
    distorted_image = cv2.imread(image_path)
    if distorted_image is None:
        raise ValueError(f"Image not found at path: {image_path}")

    # Perform undistortion
    undistorted_image = cv2.undistort(
        distorted_image,
        camera_matrix,
        dist_coeffs,
        None,
        camera_matrix  # Use the same camera matrix for the new camera matrix
    )

    # Save the undistorted image
    cv2.imwrite(output_path, undistorted_image)
    print(f"Undistorted image saved to: {output_path}")


def process_images_in_sequence(input_dir, output_dir_5_params, output_dir_8_params, camera_matrix_5_params, dist_coeffs_5_params, camera_matrix_8_params, dist_coeffs_8_params):
    """
    Processes all images in a directory in sequence and saves both 5-parameter and 8-parameter undistorted images.

    Parameters:
        input_dir (str): Directory containing input distorted images.
        output_dir_5_params (str): Directory to save undistorted images (5-parameter model).
        output_dir_8_params (str): Directory to save undistorted images (8-parameter model).
        camera_matrix_5_params (np.ndarray): Camera intrinsic matrix for 5-parameter model.
        dist_coeffs_5_params (np.ndarray): Distortion coefficients for 5-parameter model.
        camera_matrix_8_params (np.ndarray): Camera intrinsic matrix for 8-parameter model.
        dist_coeffs_8_params (np.ndarray): Distortion coefficients for 8-parameter model.
    """
    # Create the output directories if they don't exist
    os.makedirs(output_dir_5_params, exist_ok=True)
    os.makedirs(output_dir_8_params, exist_ok=True)

    # List all files in the input directory and filter for supported image formats
    image_files = [f for f in os.listdir(input_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]

    if not image_files:
        print(f"No images found in the input directory: {input_dir}")
        return

    # Sort the image files to ensure sequential processing
    image_files.sort()  # Sort alphabetically/numerically

    # Process each image in sequence
    for image_file in image_files:
        input_image_path = os.path.join(input_dir, image_file)

        # Define output paths for 5-parameter and 8-parameter models
        output_image_path_5_params = os.path.join(output_dir_5_params, image_file)
        output_image_path_8_params = os.path.join(output_dir_8_params, image_file)

        try:
            # Undistort using 5-parameter model
            # undistort_image(input_image_path, camera_matrix_5_params, dist_coeffs_5_params, output_image_path_5_params)

            # Undistort using 8-parameter model
            undistort_image(input_image_path, camera_matrix_8_params, dist_coeffs_8_params, output_image_path_8_params)

        except Exception as e:
            print(f"Error processing image {image_file}: {e}")


# Example usage
if __name__ == "__main__":
    # Camera intrinsic matrices for 5-parameter and 8-parameter models
    camera_matrix_5_params = np.array([
        [1417.541654, 0, 2089.904695],  # fx, 0, cx
        [0, 1418.002061, 1235.591032],  # 0, fy, cy
        [0, 0, 1]                       # 0, 0, 1
    ])

    camera_matrix_8_params = np.array([
        [1419.488223, 0, 2091.72389],  # fx, 0, cx
        [0, 1419.783836, 1231.71474],  # 0, fy, cy
        [0, 0, 1]                       # 0, 0, 1
    ])

    # Distortion coefficients (k1, k2, p1, p2, k3, [k4, k5, k6])
    dist_coeffs_5_params = np.array([-0.2972345646, 0.08469466538, -0.0004248149362, -0.000175399726, 0.0])  # 5-parameter model
    dist_coeffs_8_params = np.array([0.4765817545, 0.0267510319, 1.495544387e-07, -2.581193063e-07, 8.833337814e-05, 0.8074516265, 0.1103335164, 0.001695886702])  # 8-parameter model

    # Input and output directories
    input_directory = "/media/kodifly/Extreme SSD/isds_sample_data_0515/fisheye_image"
    output_directory_5_params = "/media/kodifly/Extreme SSD/isds_sample_data_0515/tmp"
    output_directory_8_params = "/media/kodifly/Extreme SSD/isds_sample_data_0515/rectified_image"

    # Process images in sequence and save both 5-parameter and 8-parameter undistorted images
    process_images_in_sequence(
        input_directory,
        output_directory_5_params,
        output_directory_8_params,
        camera_matrix_5_params,
        dist_coeffs_5_params,
        camera_matrix_8_params,
        dist_coeffs_8_params
    )