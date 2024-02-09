import json
import numpy as np
import os



def convert_to_serializable(obj):
    """
    Convert NumPy arrays to Python lists for JSON serialization.

    Parameters:
    - obj: Object to convert

    Returns:
    - Serialized object
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    return obj


def create_json_file(file_name, camera_data, joint_angles, keypoints_data):
    """
    Create a JSON file and fill it with given data.

    Parameters:
    - file_name (str): The name of the JSON file to be created.
    - camera_data (dict): Dictionary containing camera data with location and quaternion in world frame.
    - joint_angles (list or np.array): List or NumPy array of joint angles of the robot.
    - keypoints_data (list): List of dictionaries containing keypoints with 3D and 2D coordinates.

    Returns:
    - None
    """
    data = {
        "camera_data": camera_data,
        "joint_angles": convert_to_serializable(joint_angles),
        "keypoints_data": convert_to_serializable(keypoints_data)
    }

    with open(file_name, 'w') as json_file:
        json.dump(data, json_file, default=convert_to_serializable, indent=4)

        

if __name__ == '__main__':
    # Example usage:
    num_cameras = 2
    keypoints = []
    keypoint_name = "keypoint_3"
    joint_angles = np.array([30.0, 45.0, 60.0, 90.0, 120.0])
    camera_data = {
        "location_worldframe": np.array([-75.051300048828125, 47.982898712158203, 91.198799133300781]),
        "quaternion_xyzw_worldframe": np.array([0.047899998724460602, 0.078100003302097321, -0.52090001106262207, 0.84869998693466187])
    }
    keypoint3D = np.array([2.27950513e-05, -9.52929258e-06, 5.35422587e-05])
    keypoint2D = np.array([646.28174275, 158.75526004])
    keypoints.append((keypoint_name, keypoint3D, keypoint2D))
    keypoints.append((keypoint_name, keypoint3D, keypoint2D))

    keypoints_data = [
        {"name": "keypoint1", "location": np.array([1.0, 2.0, 3.0]), "projected_location": np.array([100, 200])},
        {"name": "keypoint2", "location": np.array([4.0, 5.0, 6.0]), "projected_location": np.array([150, 250])},
        # Add more keypoints as needed
    ]
    keypoints_data.append({"name": keypoint_name, "location": keypoint3D, "projected_location": keypoint2D})
    print(keypoints_data)

    file_path = os.path.join(os.getcwd(), "json_file.json")

    create_json_file(file_path, camera_data, joint_angles, keypoints_data)