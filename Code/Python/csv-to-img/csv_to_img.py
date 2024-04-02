import csv
import os
import matplotlib.pyplot as plt

# Set the path to the folder containing the CSV files
folder_path = os.path.join('.', 'csvs')

# Loop through all the files in the folder
for file in os.listdir(folder_path):
    # Check if the file is a CSV file
    if file.endswith('.csv'):
        # Open the CSV file and read the X and Y coordinates
        with open(os.path.join(folder_path, file), 'r') as f:
            reader = csv.reader(f, delimiter=';')
            x_coords = []
            y_coords = []
            for row in reader:
                x_coords.append(float(row[0].replace(',', '.')))
                y_coords.append(float(row[1].replace(',', '.')))

        # Create a new figure
        plt.figure()

        # Plot the points as circles
        plt.plot(x_coords, y_coords, 'o')

        # Remove the tick marks
        plt.xticks([])
        plt.yticks([]) 

        # Remove the spines
        ax = plt.gca()
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)

        output_folder_path = os.path.join('.', 'images')

        # Save the plot as an image with the same

