
arr = [
    [0,-1,0,0],
    [-1,0,-1,0],
    [0,0,0,0]]

def fill(arr):
    for row in arr:
        f_indexes = []
        s_indexes = []
        flag = False
        for index in range(len(row)):
            if row[index] == -1 and not flag:
                f_indexes.append(index)
                flag = True
            elif row[index] == -1 and flag and row[index+1] == 0:
                s_indexes.append(index)
                flag = False

        if len(f_indexes) == len(s_indexes):
            for i,k in zip(f_indexes,s_indexes):
                for j in range(i,k+1):
                    row[j] = -1


fill(arr)
print(arr)
