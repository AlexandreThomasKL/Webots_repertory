import string
import os
import random
import time
from pathlib import Path

#path_file_coord = r'd:\Dossiers locaux\Bureau\Scolaire\ESIREM 2021-20XX\3ème semestre\stage\Stage_subject\Webots_project\Swarm_test\controllers\ship_swarm_quantumGate_instant\position_indicator.txt'
path_dir = os.getcwd()
file_name = "\position_indicator"
name_variable_file = "\static_variables.txt"
extension_end = ".txt"


path_file_coord = Path(path_dir + file_name + extension_end)

# Varaible to 
lenght_area = 15.0
nb_robot = 2

# Check if the path exist, return a bool
def FileCheck(path_file):
    try:
        open(path_file,"r")
        return True
    except:
        print ("Error : file not found or exist")
        return False

# Define a list of robot name
def list_name_robot(nb_robot):
    name_str =[]
    name_str.append("ship_swarm_data.txt")
    for idx in range(1,nb_robot):
        name_str.append("ship_swarm({})_data.txt".format(idx))

    return name_str


def main ():

    #--- Variable file ---

    print("--- Static variables ---")
    path_file_var = path_dir +  name_variable_file
    state_file = FileCheck(path_file_var)
    # Create the variables file if it doesn't exist, otherwise format the actual one
    if state_file == True:
        print ("File {} already exist, reset to blank file...".format(name_variable_file))        
    else:
        print ("File {} doesn't exist, creation...".format(name_variable_file))

    with open(path_file_var,"w") as fileDataVar:
        fileDataVar.close()

    # Until something is not written in the varaible file, wait
    with open(path_file_var,"r") as fileDataVar:
        var_fill = True
        while var_fill:
            line_rob_disp = fileDataVar.readlines()
            var_fill = not bool(line_rob_disp)
            print("Waiting text variable to be fill...")
            time.sleep(10)
        print("Number of robot:",len(line_rob_disp))


    # Check if the files referencing the positions exist
    print("--- Files location references ---")
    for idx in range(nb_robot):
        global_name_file = file_name + "_" + str(idx) + extension_end
        path_file_coord = Path(path_dir + global_name_file)
        state_file = FileCheck(path_file_coord)
        if state_file == True:
            print ("File {} already exist, reset to blank file...".format(global_name_file))
            
        else:
            print ("File {} doesn't exist, creation...".format(global_name_file))
            
        with open(path_file_coord,"w") as fileData:
                fileData.close()


    # Wait utlie the robot files are created
    print("--- Files robot location ---")
    list_name = list_name_robot(nb_robot)
    creation_file = False
    while (creation_file == False):
        count_rob = 0
        for name in list_name:
            if FileCheck(path_dir + "\{}".format(name)):
                print(name,"exist in the folder")
                count_rob += 1
            else :
                print("Waiting for file {} to be create...".format(name))
                time.sleep(3)
        
            if count_rob == len(list_name):
                creation_file = True


    print("--- Writting location ---")
    count_pos = 0
    while (count_pos < 3):
        count_pos += 1
        
        # Write an random position to the robot to go
        for idx in range(nb_robot):
            global_name_file = file_name + "_" + str(idx) + extension_end
            path_file_coord = Path(path_dir + global_name_file)
            #print(global_name_file ,"writting :",count_pos)

            # Write a random position
            with open(path_file_coord,"a+") as fileData:
                x_value,y_value = round(random.uniform(0.1, 0.9)*lenght_area,1),round(random.uniform(0.1, 0.9)*lenght_area,1)
                line_add =  str(x_value) + ";" + str(y_value) + "\n"
                fileData.write(line_add)
                fileData.close()

            # Read the last position
            path_file_coord_rob = path_dir + "\{}".format(list_name[idx])
            #print(list_name[idx],"reading :",count_pos)
            with open(path_file_coord_rob,"r") as fileDataRob:
                line_pos_rob = fileDataRob.readlines()
                line_strip = line_pos_rob[-1].strip()
                pos_coord = [coord for coord in line_strip.split(';')]
                print(pos_coord)
            
        time.sleep(4)

if __name__ == "__main__":
    main()

