closed_list = dict()
start_loc = (3,2)
h_value = 10
root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, "timestep": 0}

closed_list[(root['loc'], root['timestep'])] = root

# child = {'loc': (3,3), 'g_val': 0, 'h_val': h_value, 'parent': None, "timestep": 1}


# closed_list[(child['loc'])] = tuple((child['loc'], child['timestep']))

hi = []
for j in range (4):
    for i in range(4):
        if i == 2:
            break
        print("i is: ",i)
    print("j is: ", j)