import sys

# Gets the temp name passed in.
if len(sys.argv) != 2:
	print("You must supply a valid filename to continue.")
	print("Usage: " + sys.argv[0] + " " + "filename.txt")

	sys.exit(1)

# Opens up the temporary file and processes it.
with open(sys.argv[1]) as f:
    content = f.readlines()
content = [x.strip() for x in content] 

# Loop through the triage file and splits by space.
print("\nFACT TUPLE :")
for line in content:
	delim = line.split(' ')
	
	# Take the two and generate an ID.
	ID = delim[1] + delim[2]
	size = delim[0]
	
	#Print out the entry.
	print("pathSize " + ID + " " + size);
