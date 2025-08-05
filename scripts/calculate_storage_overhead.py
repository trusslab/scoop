import subprocess
import sys
import os

def main():
    
    dataset_path = "../../dataset/"

    data_points = os.listdir(dataset_path)
    data_points.sort()
    
    num_of_datapoints = 0

    ios_overhead_sum = 0.0
    android_overhead_sum = 0.0

    ios_overhead_max = 0.0
    ios_overhead_min = 100.0
    android_overhead_max = 0.0
    android_overhead_min = 100.0

    for data_point in data_points:
        try:
            data_id = int(data_point.split("_")[0])
        except:
            continue

        num_of_datapoints += 1

        ios_original_size = 0
        ios_depth_map_size = 0

        android_original_size = 0
        android_depth_map_size = 0

        files_in_data_point = os.listdir(dataset_path + data_point)
        for file in files_in_data_point:
            if "ios" in file and "photo" in file:
                # if it's end with .jpg, add to original size
                if file.endswith(".jpg"):
                    ios_original_size += os.path.getsize(dataset_path + data_point + "/" + file)
                elif file.endswith(".idep"):
                    ios_depth_map_size += os.path.getsize(dataset_path + data_point + "/" + file)
            elif "android" in file and "photo" in file:
                if file.endswith(".jpg"):
                    android_original_size += os.path.getsize(dataset_path + data_point + "/" + file)
                elif file.endswith(".adep"):
                    android_depth_map_size += os.path.getsize(dataset_path + data_point + "/" + file)

        ios_overhead = ios_depth_map_size / ios_original_size
        android_overhead = android_depth_map_size / android_original_size

        ios_overhead_max = max(ios_overhead_max, ios_overhead)
        ios_overhead_min = min(ios_overhead_min, ios_overhead)
        android_overhead_max = max(android_overhead_max, android_overhead)
        android_overhead_min = min(android_overhead_min, android_overhead)

        ios_overhead_sum += ios_overhead
        android_overhead_sum += android_overhead

    ios_overhead_avg = ios_overhead_sum / num_of_datapoints
    android_overhead_avg = android_overhead_sum / num_of_datapoints

    print("Average iOS storage overhead: ", ios_overhead_avg)
    print("Average Android storage overhead: ", android_overhead_avg)

    print("Max iOS storage overhead: ", ios_overhead_max)
    print("Min iOS storage overhead: ", ios_overhead_min)
    print("Max Android storage overhead: ", android_overhead_max)
    print("Min Android storage overhead: ", android_overhead_min)

    # now calculate the MSEs
    ios_mse = 0.0
    android_mse = 0.0

    for data_point in data_points:
        try:
            data_id = int(data_point.split("_")[0])
        except:
            continue

        ios_original_size = 0
        ios_depth_map_size = 0

        android_original_size = 0
        android_depth_map_size = 0

        files_in_data_point = os.listdir(dataset_path + data_point)
        for file in files_in_data_point:
            if "ios" in file and "photo" in file:
                # if it's end with .jpg, add to original size
                if file.endswith(".jpg"):
                    ios_original_size += os.path.getsize(dataset_path + data_point + "/" + file)
                elif file.endswith(".idep"):
                    ios_depth_map_size += os.path.getsize(dataset_path + data_point + "/" + file)
            elif "android" in file and "photo" in file:
                if file.endswith(".jpg"):
                    android_original_size += os.path.getsize(dataset_path + data_point + "/" + file)
                elif file.endswith(".adep"):
                    android_depth_map_size += os.path.getsize(dataset_path + data_point + "/" + file)

        ios_mse += (ios_depth_map_size / ios_original_size - ios_overhead_avg) ** 2
        android_mse += (android_depth_map_size / android_original_size - android_overhead_avg) ** 2

    ios_mse /= num_of_datapoints
    ios_mse = ios_mse ** 0.5
    android_mse /= num_of_datapoints
    android_mse = android_mse ** 0.5

    print("iOS MSE: ", ios_mse)
    print("Android MSE: ", android_mse)

if __name__ == "__main__":
    main()


    
    

