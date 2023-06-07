import glob, os
import sys
import fileinput, re
from subprocess import call

#print ('test', len(sys.argv))
#print (str(sys.argv[1]))

for root, dirs, files in os.walk(sys.argv[1]):
    for file in files:
        if file.endswith(".scn"):
            target = "fileMesh"
            filePath = os.path.join(root, file)

            # access header and store all includes            
            with open(filePath, 'r') as thefile:
                data = thefile.readlines()                        
            
            includes = {}
            target2 = "OglModel"
            
            for idx, line in enumerate(data):
                if re.search(target, line) and re.search(target2, line):
                    if re.search(".obj", line):
                        includes[idx] = line
                    else:
                        print(file, line) 
                        
                        
            if (not includes):
                continue
                
                
            # create backup datafile
            dataBackup = data[:]
            print(filePath)
            
            cptLoader = 0
            
            # for each component with fileMesh
            for key, value in includes.items():
                #data[key] = "\n" + value
                print("----------------")
                print("Before: ", key, value)
                
                # create the line of the meshLoader
                indent = value.split("<")
                indent = indent[0]
                loaderName = "meshLoader_" + str(cptLoader)
                loaderLine = indent + '<MeshObjLoader name="' + loaderName + '" '                 
                cptLoader = cptLoader+1;
                
                # split value into field values
                values = value.split('"')
                skip = False
                newLine=""
                
                # parse the values
                for idx, field in enumerate(values):
                    if skip == True:
                        skip = False
                        continue
                        
                    if re.search("translation", field) or re.search("rotation", field) or re.search("scale", field):
                        if re.search("<OglModel", field):
                            newLine = newLine + "<OglModel"
                        loaderLine = loaderLine + field + '"' + values[idx+1] + '"'
                        skip = True
                    elif re.search("fileMesh", field):
                        if re.search("<OglModel", field):
                            newLine = newLine + "<OglModel"
                        skip = True
                        
                        newLine = newLine + ' src="@' + loaderName + '"'
                        loaderLine = loaderLine + 'filename="' + values[idx+1] + '"'
                    elif re.search("=", field) or re.search("/>", field):
                        newLine = newLine + field
                    else: # need to restore the " " for the fields
                        newLine = newLine + '"' + field + '"'                   
                
                # add ending of meshloader line
                loaderLine = loaderLine + " />"
                
                print("After: \n", loaderLine, "\n" , newLine)
                print("----------------")
                
                # overwrite existing line
                data[key]  = loaderLine + "\n" + newLine

            with open(filePath, 'w') as newfile:
                newfile.writelines(data)
                    
                #res = call(["msbuild.exe", "C:\projects\sofa-build\ALL_BUILD.vcxproj"], shell=True)
            #    res = call(["msbuild.exe", sys.argv[2]], shell=True)
            #    if res == 0:
            #        dataBackup = data[:]
            #    else:
            #        data = dataBackup[:]

            #with open(filePath, 'w') as newfile:
            #    newfile.writelines(data)                    
            
          
                    