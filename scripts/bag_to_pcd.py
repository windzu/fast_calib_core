#!/usr/bin/env python3
"""
Accumulate Point Clouds from ROS2 Bag Files

This script reads point cloud data from ROS2 bag files (mcap or db3 format)
and saves them as PCD files for use with fast_calib_core calibration.

Usage:
    python3 bag_to_pcd.py <bag_dir> [options]

Examples:
    # Extract from all bags in a directory using default topic
    python3 bag_to_pcd.py /path/to/bags

    # Specify a custom topic
    python3 bag_to_pcd.py /path/to/bags --topic /velodyne_points

    # Use binary PCD format for all files
    python3 bag_to_pcd.py /path/to/bags --binary

    # Specify output directory
    python3 bag_to_pcd.py /path/to/bags --output /path/to/output

Dependencies:
    - rclpy
    - rosbag2_py
    - numpy
"""

import argparse
import struct
import numpy as np
from pathlib import Path

# ROS2 imports
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


def read_points_from_pointcloud2(msg):
    """
    Extract XYZI points from a PointCloud2 message.
    Returns a numpy array of shape (N, 4) with float32 values (x, y, z, intensity).
    """
    # Parse the point cloud fields to find x, y, z, intensity offsets
    field_names = [f.name for f in msg.fields]
    field_offsets = {f.name: f.offset for f in msg.fields}

    # Check if we have xyz fields
    if "x" not in field_names or "y" not in field_names or "z" not in field_names:
        raise ValueError("PointCloud2 message doesn't contain x, y, z fields")

    # Get offsets
    x_offset = field_offsets["x"]
    y_offset = field_offsets["y"]
    z_offset = field_offsets["z"]

    # Check for intensity field
    has_intensity = "intensity" in field_names
    if has_intensity:
        intensity_offset = field_offsets["intensity"]

    point_step = msg.point_step
    width = msg.width
    height = msg.height

    # Total number of points
    num_points = width * height

    # Extract data
    data = np.frombuffer(msg.data, dtype=np.uint8)

    points = []
    for i in range(num_points):
        offset = i * point_step
        x = struct.unpack_from("<f", data, offset + x_offset)[0]
        y = struct.unpack_from("<f", data, offset + y_offset)[0]
        z = struct.unpack_from("<f", data, offset + z_offset)[0]

        if has_intensity:
            intensity = struct.unpack_from("<f", data, offset + intensity_offset)[0]
        else:
            intensity = 0.0

        # Skip invalid points (NaN or Inf)
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            points.append([x, y, z, intensity])

    return np.array(points, dtype=np.float32)


def save_pcd_binary(points: np.ndarray, output_path: str):
    """
    Save points to PCD file format (binary).

    Args:
        points: numpy array of shape (N, 4) with x, y, z, intensity
        output_path: path to output PCD file
    """
    num_points = len(points)

    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA binary
"""

    with open(output_path, "wb") as f:
        f.write(header.encode("utf-8"))
        points.astype(np.float32).tofile(f)

    print(f"Saved {num_points} points to {output_path} (binary format)")


def save_pcd_ascii(points: np.ndarray, output_path: str):
    """
    Save points to PCD file format (ASCII).

    Args:
        points: numpy array of shape (N, 4) with x, y, z, intensity
        output_path: path to output PCD file
    """
    num_points = len(points)

    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA ascii
"""

    with open(output_path, "w") as f:
        f.write(header)
        for pt in points:
            f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f} {pt[3]:.6f}\n")

    print(f"Saved {num_points} points to {output_path} (ascii format)")


def extract_pointcloud_from_bag(bag_path: str, topic: str):
    """
    Extract all point clouds from a rosbag for a specific topic.

    Args:
        bag_path: path to the rosbag (.mcap or .db3 file)
        topic: topic name to extract

    Returns:
        numpy array of all points from this bag
    """
    # Determine storage id based on file extension
    bag_path_obj = Path(bag_path)
    if bag_path_obj.suffix == ".mcap":
        storage_id = "mcap"
    elif bag_path_obj.suffix == ".db3":
        storage_id = "sqlite3"
    else:
        # Try mcap by default
        storage_id = "mcap"

    # Setup reader
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {meta.name: meta.type for meta in topic_types}

    print(f"  Available topics: {list(type_map.keys())}")

    if topic not in type_map:
        print(f"  Warning: Topic {topic} not found in {bag_path}")
        return np.array([], dtype=np.float32).reshape(0, 4)

    # Get message type class
    msg_type = get_message(type_map[topic])

    # Read all messages from the topic
    all_points = []
    msg_count = 0

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()

        if topic_name == topic:
            msg = deserialize_message(data, msg_type)
            points = read_points_from_pointcloud2(msg)
            if len(points) > 0:
                all_points.append(points)
            msg_count += 1

    print(f"  Read {msg_count} messages from topic {topic}")

    if not all_points:
        return np.array([], dtype=np.float32).reshape(0, 4)

    # Combine all points
    combined_points = np.vstack(all_points)
    print(f"  Total points from this bag: {len(combined_points)}")

    return combined_points


def main():
    parser = argparse.ArgumentParser(
        description="Extract point clouds from ROS2 bag files and save as PCD files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s /path/to/bags                    # Use default topic /livox/lidar
  %(prog)s /path/to/bags --topic /velodyne  # Custom topic
  %(prog)s /path/to/bags --binary           # Force binary PCD format
  %(prog)s /path/to/bags -o /output/dir     # Custom output directory
        """,
    )
    parser.add_argument(
        "bag_dir",
        type=str,
        help="Directory containing ROS2 bag files (.mcap or .db3)",
    )
    parser.add_argument(
        "--topic",
        "-t",
        type=str,
        default="/livox/lidar",
        help="Point cloud topic name (default: /livox/lidar)",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default=None,
        help="Output directory for PCD files (default: same as bag_dir)",
    )
    parser.add_argument(
        "--binary",
        "-b",
        action="store_true",
        help="Force binary PCD format for all files",
    )
    parser.add_argument(
        "--ascii",
        "-a",
        action="store_true",
        help="Force ASCII PCD format for all files",
    )

    args = parser.parse_args()

    data_dir = Path(args.bag_dir)
    if not data_dir.exists():
        print(f"Error: Directory {data_dir} does not exist")
        return 1

    output_dir = Path(args.output) if args.output else data_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    topic = args.topic

    # Find all bag files
    bag_files = sorted(data_dir.glob("*.db3")) + sorted(data_dir.glob("*.mcap"))

    if not bag_files:
        print(f"No .mcap or .db3 files found in {data_dir}")
        return 1

    print(f"Found {len(bag_files)} bag files:")
    for f in bag_files:
        print(f"  - {f.name}")
    print(f"\nTopic: {topic}")
    print(f"Output: {output_dir}\n")

    # Process each bag file separately and save to individual PCD
    success_count = 0
    for bag_file in bag_files:
        print(f"Processing: {bag_file.name}")
        points = extract_pointcloud_from_bag(str(bag_file), topic)

        if len(points) == 0:
            print(f"  No points extracted, skipping...")
            print()
            continue

        # Generate output filename based on bag filename
        output_pcd = output_dir / f"{bag_file.stem}.pcd"

        # Determine format
        if args.binary:
            save_pcd_binary(points, str(output_pcd))
        elif args.ascii:
            save_pcd_ascii(points, str(output_pcd))
        else:
            # Auto: use binary for large point clouds
            if len(points) > 1000000:
                save_pcd_binary(points, str(output_pcd))
            else:
                save_pcd_ascii(points, str(output_pcd))

        success_count += 1
        print()

    print("=" * 50)
    print(f"Done! Saved {success_count}/{len(bag_files)} PCD files to: {output_dir}")
    return 0


if __name__ == "__main__":
    exit(main())
