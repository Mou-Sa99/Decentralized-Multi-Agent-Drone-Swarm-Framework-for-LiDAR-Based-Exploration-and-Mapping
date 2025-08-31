import json
import os



def write_point_to_json(point, file_path):
    """
    Append a 3D point to a JSON file with index.

    Args:
        point: tuple or list (x, y, z)
        file_path: path to the JSON file
    """
    # Ensure directory exists
    os.makedirs(os.path.dirname(file_path), exist_ok=True)

    # If file exists, load it
    if os.path.exists(file_path):
        with open(file_path, "r") as f:
            data = json.load(f)
    else:
        data = {}

    # Determine next index
    next_index = str(len(data))

    # Store point (convert to list for JSON compatibility)
    data[next_index] = list(point)

    # Write back to file
    with open(file_path, "w") as f:
        json.dump(data, f, indent=4)

    print(f"✅ Point {point} written at index {next_index}")
