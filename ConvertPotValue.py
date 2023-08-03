import pandas as pd

# Specify the path to your CSV file
csv_file_path = "C:/FYP/Lane Detection/Data_back up/framezx.csv"

# Read the CSV file into a DataFrame
df = pd.read_csv(csv_file_path)

# Get the sensor column as a Series
sensor_column = df.iloc[:, 1]  # Replace 1 with the correct column index
max = sensor_column.max()
min = sensor_column.min()
print(max)
print(min)
# Perform normalization using Min-Max Scaling to range [-1, 1]
normalized_column = (sensor_column - sensor_column.min()) / (sensor_column.max() - sensor_column.min()) * 2 - 1

# Replace the original sensor column with the normalized values
df.iloc[:, 1] = normalized_column  # Replace 1 with the correct column index

# Save the modified DataFrame back to CSV
df.to_csv("C:/FYP/Lane Detection/Data_back up/framezx.csv", index=False)
# sensor_column = data.iloc[:, 1]  # Replace 1 with the correct column index (Performed once only)
# min_sensor= sensor_column.min()
# max_sensor= sensor_column.max()
# Perform normalization using Min-Max Scaling to range [-1, 1]
#normalized_column = (sensor_column - sensor_column.min()) / (sensor_column.max() - sensor_column.min()) * 2 - 1(performed once only)

# Replace the original sensor column with the normalized values (performed once only)
# data.iloc[:, 1] = normalized_column  # Replace 1 with the correct column index(performed once only)

# Save the modified DataFrame back to CSV
#data.to_csv("C:/FYP/Lane Detection/Data_back up/framezz.csv", index=False) ``
