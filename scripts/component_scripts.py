import glob, os
import sys
import fileinput, re
from subprocess import call

#print ('test', len(sys.argv))
target = str(sys.argv[2])
value = str(sys.argv[3])
print("## Replace: '", target, "' by '", value, "' in: ", sys.argv[1])

for root, dirs, files in os.walk(sys.argv[1]):
    for file in files:
        if file.endswith(".scn"):
            filePath = os.path.join(root, file)

            # access header and store all includes
            with open(filePath, 'r') as thefile:
                data = thefile.readlines()
            thefile.close()
            
            new_file = open(filePath, "w")
            for idx, line in enumerate(data):
                if re.search(target, line):
                    line = line.replace(target, value)
                    print("found: ", target, " in : ", filePath)
                new_file.write(line)
             
            new_file.close()
            