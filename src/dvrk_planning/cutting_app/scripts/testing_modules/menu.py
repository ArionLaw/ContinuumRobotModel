
########################################################
#Pre-defined goals menu
########################################################
def goal_gen(inp_data):
    goal0  = [[0.0637,0.9454,0.3197,0.0261],[0.9977,-0.0534,-0.0409,0.0395],[-0.0216,0.3216,-0.9466,-0.1045],[0,0,0,1]]
    goal1  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    goal2  = [[0, 1, 0, 0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal3  = [[0, 1, 0, 0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal4  = [[0, 1, 0, -0.05],[1, 0, 0, 0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal5  = [[0, 1, 0, -0.05],[1, 0, 0, -0.05],[0, 0, -1, -0.1],[0,0,0,1]]
    goal6  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    goal7  = [[0, 1, 0, 0.03],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    goal8  = [[0.0637,0.9454,0.3197,0],[0.9977,-0.0534,-0.0409,0],[-0.0216,0.3216,-0.9466,-0.1],[0,0,0,1]]

    #pick and place
    # goal0  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal1  = [[0, 1, 0, 0],[1, 0, 0, -0.05],[0, 0, -1, -0.13],[0,0,0,1]]
    # goal2  = [[0, 1, 0, 0],[1, 0, 0, -0.03],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal3  = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.07],[0,0,0,1]]
    # goal4  = [[0, 1, 0, 0],[1, 0, 0, 0.03],[0, 0, -1, -0.1],[0,0,0,1]]
    # goal5  = [[0, 1, 0, 0],[1, 0, 0, 0.05],[0, 0, -1, -0.13],[0,0,0,1]]
    # goal6 = []
    # goal7 = []
    # goal8 = []

    goals = [goal0, goal1, goal2, goal3, goal4, goal5, goal6, goal7, goal8]
    index = 0
    mode = 0
    try:
        output = [ [goals[int(inp_data[0])]] ]

        if (inp_data[1]=='e'):
            #print (inp_data)
            return output

        for i in range(1,len(inp_data)-1):
            
            if inp_data[i]=='.':
                mode = 0 
            elif inp_data[i]==',':
                mode = 1

            elif mode == 1:
                output[index].append( goals[int(inp_data[i])] )
            elif mode == 0:
                output[index].append( goals[int(inp_data[i])] )
                output.append ([ goals[int(inp_data[i])] ])
                index = index + 1

    except Exception as e: 
        print(e)
        menu()
    return output 

def menu():
    if rospy.is_shutdown():
        return
    
    print("separate goals with '.' and waypoints with ','" )
    print("points are 0-9")
    print("o/c for opening/closing jaw")
    data = ''
    mode = 0
    results = list(input ())
    results.append('e')
    #print (results)
    if rospy.is_shutdown():
        exit()
    if results[0]=='o' or results[0]=='c':
        data = results[0]
        mode = 1
    elif results[1]=='.' or results[1]==',' or results[1]=='e':
        data = goal_gen (results)
        mode = 2
    else:
        menu()
    #results = list(map(int, results))
    print ("Using the following data for goal generation....")
    print(data)
    return data, mode
########################################################
