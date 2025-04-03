import requests
import json
import logging

def send_json_payload(json_payload):  # Removed `self`
    headers = {"Content-Type": "application/json"}
    BACKEND_URL = "https://isds.kodifly.com"
    endpoint = "/api/ops/socket-message/"
    try:
        URL = BACKEND_URL
        response = requests.post(
            URL + endpoint, data=json.dumps(json_payload), headers=headers
        )
        logging.info(
            f"Inside send_json_payload {response} {json.dumps(json_payload)}"
        )

        if response.status_code == 201:
            return True
        else:
            logging.error(
                f"Failed to send JSON payload. Status code: {response.status_code}, Response: {response.text}"
            )
            return False

    except Exception as e:
        logging.error(
            f"Failed to send JSON payload. Error code: {e}", exc_info=True
        )
        return False
    
if __name__ == "__main__":
    # Define the JSON payload
    json_payload = {
        "message_id": "7621d54d-9c18-4b5d-a200-84d5da200fdf",
        "message_type": "GNSS_IMU_DATA",
        "sender": "gnss_imu_sensor",
        "message": {
            "gnss_data": {
                "timestamp": 1738745910.1224003,
                "latitude": 22.4140283,
                "longitude": 114.2132712,
                "altitude": 13178.0
            },
            "imu_data": {
                "timestamp": 1738745910.107247,
                "orientation_x": 0.0,
                "orientation_y": 0.0,
                "orientation_z": 0.0,
                "orientation_w": 1.0,
                "angular_velocity_x": -0.053662695965096645,
                "angular_velocity_y": 0.0011984224905356572,
                "angular_velocity_z": -0.22490395405719166,
                "linear_acceleration_x": 0.7110778930664062,
                "linear_acceleration_y": -0.38307226562499996,
                "linear_acceleration_z": 9.528922607421874
            }
        }
    }

    # Send the JSON payload
    success = send_json_payload(json_payload)  # Correctly passes `json_payload`
    if success:
        print("JSON payload sent successfully!")
    else:
        print("Failed to send JSON payload.")