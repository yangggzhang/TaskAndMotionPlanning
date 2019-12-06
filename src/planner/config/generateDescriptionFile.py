# -*- coding: utf-8 -*-
import yaml
import io
import itertools

# global parameters, load from somewhere later
saveDescriptionFile = "./test.txt"
goalCondition =  "Get(cylinder3)"
action = "Pick"
action_args = ["x"]

def generateConditions(condition, num_args, object_list):
    
    condition_arguments = list(itertools.product(object_list,repeat=num_args))
    condition_string = ""
    for i in range(len(condition_arguments)):
        single_condition = condition+"("
        for j, args in enumerate(condition_arguments[i]):
            single_condition += args
            if j < len(condition_arguments[i])-1:
                single_condition += ","
        condition_string += (single_condition+")")
        if i < len(condition_arguments)-1:
            condition_string += ", "    
    return condition_string
def generateActions(action, args, movable_list):
    #hard code for now
    if (action=="Pick"):
        assert (len(args) == 1)
        action_string = "        Pick("+args[0]+")\n"
        action_string += "        Preconditions: "
        for i, movable_obj in enumerate(movable_list):
            action_string+=("NotBlock("+movable_obj+","+args[0]+"), ")
        action_string+="Exist("+args[0]+")\n"
        action_string+="        Effects: "
        action_string+="Get("+args[0]+"), !Exist(" + args[0] + "), "
        for i, movable_obj in enumerate(movable_list):
            action_string+=("NotBlock("+args[0]+","+movable_obj+")")
            if i <len(movable_list):
                action_string+=", "

    return action_string
if __name__ == '__main__':
    # Read YAML file
    with open("CollisionObjects.yaml", 'r') as stream:
        data_loaded = yaml.safe_load(stream)

    for i in range(len(data_loaded["collision_objects"])):

        print(data_loaded["collision_objects"][i]["id"]=='table1')

    num_symbols = len(data_loaded["collision_objects"])
    movable_list = []
    all_list = []
    # Write txt
    with open(saveDescriptionFile, "w+") as text_file:
        num_symbols = len(data_loaded["collision_objects"])
        text_file.write("Symbols: ")
        for i in range(num_symbols):
            symbol = data_loaded["collision_objects"][i]["id"]
            all_list.append(symbol)
            if (data_loaded["collision_objects"][i]["movable"]==True):
                movable_list.append(symbol)
            text_file.write(symbol)
            if (i<num_symbols-1):
                text_file.write(",")
            else:
                text_file.write("\n")
        # set up initial conditions
        cond = generateConditions("NotBlock", 2, movable_list)
        print(cond)
        cond = generateConditions("Exist", 1, all_list)
        print(cond)
        text_file.write("Initial conditions: ")
        text_file.write(generateConditions("NotBlock", 2, movable_list)+", ")
        text_file.write(generateConditions("Exist", 1, all_list))
        text_file.write("\n")
        text_file.write("Goal conditions: "+goalCondition+"\n")
        text_file.write("\n")
        text_file.write("Actions:\n")
        text_file.write(generateActions(action, action_args, movable_list))



