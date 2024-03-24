import numpy as np
from rangeview_projection import rv_project


# Open the text file
# with open('/home/robotics/Downloads/kccs0310/saved_data/pcd/1710024883770776478.pcd', 'r') as file:
#     # Skip the first 7 lines
#     for _ in range(11):
#         next(file)
#
#     # Read the remaining lines and process them
#     data = []
#     for line in file:
#         # Split each line by space and convert elements to float
#         row = [float(elem) for elem in line.split()]
#         data.append(row)
#
# # Convert the list of lists into a numpy array
# array_data = np.array(data)
#
# proj = rv_project(array_data)
#
# # Print or use the numpy array as needed
# print(array_data)



import os


def rename_files_in_folder(folder_path):
    # Get a list of all files in the folder
    files = os.listdir(folder_path)

    # Iterate through each file
    for file_name in files:
        # Check if the file has the desired old extension
        if file_name[-4:] == ".txt":
            # Construct the new file name with the new extension
            new_file_name = file_name[:-4]

            # Rename the file
            os.rename(os.path.join(folder_path, file_name), os.path.join(folder_path, new_file_name))
            print(f"Renamed '{file_name}' to '{new_file_name}'")


# Provide the folder path, old extension, and new extension
folder_path = '/home/robotics/Downloads/kccs0310/saved_data/pcd'

# Call the function to rename files in the folder
rename_files_in_folder(folder_path)
