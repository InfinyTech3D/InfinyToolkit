import glob, os
import sys
import fileinput, re
from subprocess import call

#print ('test', len(sys.argv))
#print (str(sys.argv[1]))

for root, dirs, files in os.walk(sys.argv[1]):
    for file in files:
        if file.endswith(".scn"):
            filePath = os.path.join(root, file)

            # access header and store all includes            
            with open(filePath, 'r') as thefile:
                data = thefile.readlines()                        
            
            target = "OglModel"
            target2 = "SofaOpenglVisual"
            target3 = "<Node"
            
            hasOglModel = False
            requirePluginDone = False
            lineNum = -1
            
            # parsing file for oglModel
            includes = {}
            for idx, line in enumerate(data):
                if re.search(target, line):
                    hasOglModel = True
                if re.search(target2, line):
                    requirePluginDone = True
                if re.search(target3, line) and lineNum == -1:
                    lineNum = idx
                        
            if (requirePluginDone == True or hasOglModel == False):
                print("Nothing to do in", filePath)
                continue
                
            if (lineNum == -1):
                print("Error No node found in", filePath)
                continue
                
            print("Processing", filePath)
            
            # add requiredPlugin line
            newLine = '    <RequiredPlugin pluginName="SofaOpenglVisual"/>'
            nodeLine = data[lineNum] + newLine + "\n"
            #print(nodeLine)
            data[lineNum] = nodeLine
                
            with open(filePath, 'w') as newfile:
                newfile.writelines(data)
                    
