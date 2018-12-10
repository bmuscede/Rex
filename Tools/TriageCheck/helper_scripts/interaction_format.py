import sys

# Gets the temp name passed in.
if len(sys.argv) < 2:
    print("You must supply a valid filename to continue.")
    print("Usage: " + sys.argv[0] + " " + "filename.txt")

    sys.exit(1)

# Opens up the temporary file and processes it.
with open(sys.argv[1]) as f:
    content = f.readlines()
content = [x.strip() for x in content] 

# Loop through the list.
results = []
for line in content:
    delim = line.split(' -> ')
    size = len(delim) - 2
    if size == 0:
        continue

    tp = (size, delim[0], delim[len(delim) - 1])
    results.append(tp)

# Coverts to set.
results=list(set(results))

# Sorts the final results.
results= sorted(results, key=lambda tup: tup[0], reverse=True)
for line in results:
	print(str(line[0]) + " " + str(line[1]) + " " + str(line[2]))
