import sys

# Gets the temp name passed in.
if len(sys.argv) < 3:
    print("You must supply a valid rank file and result file to continue.")
    print("Usage: " + sys.argv[0] + " " + "rank_file.txt result_file.txt")

    sys.exit(1)

# Opens up the temporary rank file and processes it.
with open(sys.argv[1]) as f:
    content = f.readlines()
content = [x.strip() for x in content] 

# Loop through the rank file.
rank = {}
for line in content:
    delim = line.split(' ')
    cb = delim[len(delim) - 1]
    
    if cb not in rank:
        rank[cb] = []

    rank[cb].append(delim[0])

# Next opens the result file.
with open(sys.argv[2]) as f:
    content = f.readlines()
content = [x.strip() for x in content] 

# Loop through the results.
results = []
for line in content:
    delim = line.split(' ')
    cb = delim[0]
   
    # Find in rank.
    if cb not in rank:
        continue

    res = rank[cb]
    if len(res) == 0:
        continue

    for cur in res:
        tp = (cur, cb, delim[1])
        shouldAdd = True
        for item in results:
            if item[0] == tp[0] and item[1] == tp[1] and item[2] == tp[2]:
                shouldAdd = False
                break

        if shouldAdd:
            results.append(tp)


# Coverts to set.
results=list(set(results))

# Sorts the final results.
results= sorted(results, key=lambda tup: tup[0], reverse=True)
for line in results:
	print(str(line[0]) + " " + str(line[1]) + " " + str(line[2]))
