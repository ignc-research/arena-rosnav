import os.path as path
from pathlib import Path

dirname = path.dirname(__file__)
image_path = path.join(dirname, "../../../../arena-rosnav/simulator_setup/maps") 
csv_path = path.join(dirname, "../project_recordings")

print(dirname)
print(image_path)
print(csv_path)