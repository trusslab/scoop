import subprocess
import sys
import os
from colorama import Fore, Style
import time

def main():

    dataset_path = sys.argv[1] + '/'
    pattern_start = int(sys.argv[2])
    pattern_end = int(sys.argv[3])

    if len(sys.argv) != 4:
        print("Usage: python eval.py <dataset_path> <pattern_start> <pattern_end>")
        sys.exit(1)

    data_points = os.listdir(dataset_path)
    data_points.sort()

    # Dict: data_point: [result, time]
    final_result = {}

    for data_point in data_points:
        try:
            data_id = int(data_point.split("_")[0])
            if data_id < pattern_start or data_id > pattern_end:
                continue
        except:
            continue

        print(Fore.BLUE + "Processing: ", data_point)
        print(Style.RESET_ALL)

        # Run iOS
        ios_start_time = time.time()
        ios_command = ' '.join(['time', './Analyzer', '0', dataset_path + data_point + '/ios_photo', '1', dataset_path + data_point + '/config.ini'])
        ios_result = subprocess.run(ios_command, stdout=subprocess.PIPE, shell=True)
        ios_end_time = time.time()
        # print("iOS command: ", ios_command)
        # print(str(ios_result.stdout))
        # print(str(ios_result.stderr))
        ios_violation_detected = False
        if "Violation detected: true" in str(ios_result.stdout):
            ios_violation_detected = True
            print("Violation command: ", ios_command)
            print(str(ios_result.stdout))

        final_result[data_point + "_ios"] = [ios_violation_detected, ios_end_time - ios_start_time]
        
        # Run Android
        android_start_time = time.time()
        android_command = ' '.join(['time', './Analyzer', '0', dataset_path + data_point + '/android_photo', '0', dataset_path + data_point + '/config.ini'])
        android_result = subprocess.run(android_command, stdout=subprocess.PIPE, shell=True)
        android_end_time = time.time()
        # print("Android command: ", android_command)
        # print(str(android_result.stdout))
        # print(str(android_result.stderr))
        android_violation_detected = False
        if "Violation detected: true" in str(android_result.stdout):
            android_violation_detected = True
            print("Violation command: ", android_command)
            print(str(android_result.stdout))

        final_result[data_point + "_android"] = [android_violation_detected, android_end_time - android_start_time]
    
    print("Final Result: ")
    for key in final_result:
        print(key, final_result[key])

if __name__ == "__main__":
    main()