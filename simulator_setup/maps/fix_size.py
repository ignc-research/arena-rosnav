import os 
all_files = os.listdir('.')

for file in all_files:
    if "map_" in file and file[4].isnumeric():
        reader = open(file + "/map.yaml", "r")
        content = reader.read()
        content = content.split("\n")
        content[1] = "resolution: 0.2"
        content = "\n".join(content)
        reader.close()

        writer = open(file + "/map.yaml", "w")
        writer.write(content)
        writer.close()

    
