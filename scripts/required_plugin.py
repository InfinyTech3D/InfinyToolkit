import glob, os
import sys
import fileinput, re
from subprocess import call, run, PIPE

#arg 1: path to inspect
#arg 2: path to runSofa
print("start")
# iterate on all file in given path
for root, dirs, files in os.walk(sys.argv[1]):
    for file in files:
        if file.endswith(".scn"):
            filePath = os.path.join(root, file)
            
            #Step 1: remove all required plugin and save temporary scene
            print("file: " + file)

            # access header and store all includes            
            with open(filePath, 'r') as thefile:
                data = thefile.readlines()
            thefile.close()
                        
            target = "RequiredPlugin"
            
            new_file = open(filePath, "w")
            
            firstReq=1
            found=False
            for idx, line in enumerate(data):
                if re.search(target, line):
                    if not found:
                        firstReq = idx
                        found = True
                    continue;
                
                new_file.write(line)
             
            new_file.close()

            if found:
                firstReq = firstReq-1

            #Step 2: execute runSofa to get required plugin from inspector
            args = sys.argv[2] + " -g batch -n 1 " + filePath
            call(args)
            result = run(args, stdout=PIPE, stderr=PIPE, universal_newlines=True)
            
            #print(result.returncode, result.stdout, result.stderr)
            #Step 3: retrieve output as list of lines
            outputLines = result.stdout.splitlines( )
            
            #Step 4: get scene without required plugin and rewrite it
            with open(filePath, 'r') as thefile:
                dataTmp = thefile.readlines()
            thefile.close()
            
            newPlugs = dataTmp[firstReq]
            newPlugs = newPlugs + '    <Node name="RequiredPlugins">\n'
            
            for line in outputLines:
                if re.search("<RequiredPlugin", line):
                    newPlugs = newPlugs + "      " + line + "\n"
            
            newPlugs = newPlugs + '    </Node>\n'

            dataTmp[firstReq] = newPlugs
            #print("-----------")
            #print("newPlugs: " + newPlugs)
            
            with open(filePath, 'w') as newfile:
                newfile.writelines(dataTmp)
            
                    
