import glob, os
import sys
import fileinput, re
from subprocess import call

#print ('test', len(sys.argv))
#print (str(sys.argv[1]))

#CollisionPipeline DefaultPipeline
#CollisionResponse DefaultContactManager
#CollisionGroup DefaultCollisionGroupManager
#EulerImplicit EulerImplicitSolver
#TriangleModel TriangleCollisionModel
#LineModel LineCollisionModel
#PointModel PointCollisionModel

for root, dirs, files in os.walk(sys.argv[1]):
    for file in files:
        if file.endswith(".scn"):
            filePath = os.path.join(root, file)

            # access header and store all includes            
            with open(filePath, 'r') as thefile:
                data = thefile.readlines()                        
            
            target = "<PointModel"
            target2 = "< PointModel"
            
            # parsing file for PointModel
            includes = {}
            for idx, line in enumerate(data):
                if re.search(target, line):
                    includes[idx] = line
                if re.search(target2, line):
                    includes[idx] = line
                        
            if (not includes):
                continue
                
            # create backup datafile
            #dataBackup = data[:]
            print(filePath)
            
            # for each component with fileMesh
            for key, value in includes.items():
                #print("----------------")
                #print("Before: ", key, value)
                value.replace("<PointModel", "<PointCollisionModel")
                line = value.replace("<PointModel", "<PointCollisionModel")
                line = line.replace("< PointModel", "<PointCollisionModel")
                #print("After: ", key, line)
                #print("----------------")
                
                # overwrite existing line
                data[key] = line
                
            with open(filePath, 'w') as newfile:
                newfile.writelines(data)
                    