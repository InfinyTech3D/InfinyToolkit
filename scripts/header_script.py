import glob, os
import sys
import fileinput, re
from subprocess import call

#print ('test', len(sys.argv))
#print (str(sys.argv[1]))

for root, dirs, files in os.walk(sys.argv[1]):
    for file in files:
        if file.endswith(".s"):
            target = "#include"
            filePath = os.path.join(root, file)

            # access header and store all includes            
            with open(filePath, 'r') as thefile:
                data = thefile.readlines()                        
            
            includes = {}
            
            for idx, line in enumerate(data):
                if re.match(target, line):
                    if re.match("config.h", line):
                        continue
                    else:
                        includes[idx] = line
                    
            # create backup datafile
            dataBackup = data[:]
            print(filePath)
            for key, value in includes.items():
                data[key] = ""

                with open(filePath, 'w') as newfile:
                    newfile.writelines(data)
                    
                #res = call(["msbuild.exe", "C:\projects\sofa-build\ALL_BUILD.vcxproj"], shell=True)
                res = call(["msbuild.exe", sys.argv[2]], shell=True)
                if res == 0:
                    dataBackup = data[:]
                else:
                    data = dataBackup[:]

            with open(filePath, 'w') as newfile:
                newfile.writelines(data)                    
            
          
                    