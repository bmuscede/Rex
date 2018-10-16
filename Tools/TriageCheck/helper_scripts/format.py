import sys

# Gets the temp name passed in.
if len(sys.argv) < 3:
	print("You must supply a valid filename and ignore flag to continue.")
	print("Usage: " + sys.argv[0] + " " + "filename.txt" + "[ignore/n-ignore]")

	sys.exit(1)

ignoreFlag = False
if sys.argv[2] == "ignore":
	ignoreFlag = True
pathOutput = False
if len(sys.argv) == 4:
	pathOutput = True

# Opens up the temporary file and processes it.
with open(sys.argv[1]) as f:
    content = f.readlines()
content = [x.strip() for x in content] 

# Loop through the list.
results = []
i = -1
cbName = ""
varName = ""

for line in content:
	if line == "####":
		i = 0
	elif i == 0:
		i = i + 1
		if not ignoreFlag:
			cbName = line
	elif i == 1:
		i = -1
		if not ignoreFlag:
			varName = line
	else:		
		delim = line.split(' -> ')
		size = len(delim) - 1
	
		# Add to tuple.
		if ignoreFlag:
			if pathOutput:
				tp = (size, delim[0], delim[len(delim) - 1], line)
				results.append(tp)
			else:
				tp = (size, delim[0], delim[len(delim) - 1])
				results.append(tp)
		else:
			if pathOutput:
				tp = (size, cbName, varName, line)
				results.append(tp)
			else:
				tp = (size, cbName, varName)
				results.append(tp)

# Coverts to set.
results=list(set(results))

# Sorts the final results.
results= sorted(results, key=lambda tup: tup[0], reverse=True)
for line in results:
	print(str(line[0]) + " " + str(line[1]) + " " + str(line[2]))
	if pathOutput:
		print(str(line[3]) + "\n")
