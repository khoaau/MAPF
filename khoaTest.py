closed_list = dict()
start_loc = (3,2)
h_value = 10
root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, "timestep": 0}

closed_list[(root['loc'], root['timestep'])] = root

# child = {'loc': (3,3), 'g_val': 0, 'h_val': h_value, 'parent': None, "timestep": 1}


# closed_list[(child['loc'])] = tuple((child['loc'], child['timestep']))


constraints = [{'agent': 0,
            'loc': [(1,5)],
            'timestep':4
            }, 
            {'agent': 1,
            'loc': [(5,5)],
            'timestep':5
            }]

key,value = 'agent', 0
for i in constraints:
    if key in i and value == i[key]:
        print(i)

i = 5
j = 4
def is_constrained(j):
    if j == 4:
        return True
    return False
    
while (i > 0):
    for k in range(2):
        if is_constrained(j):
            continue
        print("i is :", i)
    print(i)
    i = i -1
    j= j -4
