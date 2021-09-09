import json
import os

# function to add to JSON
def write_json(new_data, filename='bounding_boxes.json'):
    with open(filename,'r+') as file:
          # First we load existing data into a dict.
        file_data = json.load(file)
        # Join new_data with file_data inside emp_details
        file_data["boundingBoxes"].update(new_data)
        # Sets file's current position at offset.
        file.seek(0)
        # convert back to json.
        json.dump(file_data, file, indent = 4)

    # python object to be appended

f = open('bounding_boxes.json')
data = json.load(f)

# Iterating through the json
# list
for i in data['boundingBoxes']:
    print("img:{}".format(i))
    name1="aug_g_{}".format(i)
    name2="aug_r_{}".format(i)
    aug1_out = {
        name1 : []
    }
    aug2_out = {
        name2 : []
    }
    for ii in data['boundingBoxes'][i]:
        aug1_out[name1].append(ii)
        aug2_out[name2].append(ii)
        # print(data['boundingBoxes'][i][ii])
    write_json(aug1_out)
    write_json(aug2_out)

# Closing file
f.close()
