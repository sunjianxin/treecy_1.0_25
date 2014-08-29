#!/usr/bin/python
import sys, string, os, subprocess

print "<========================================================>"
print "Treecy 1.0, RVL Lab, ECE at Purdue"
print "<========================================================>"
print "Process starts: "


'''

print "Rescale the raw tree trunk..."
subprocess.call(['rescaleTree/build/findcylinder'])
print "Done with saving the rescaled pcd file"




print "Finding Cutting Points..."
fCP = 'findCuttingPoint/build/findcylinder'
threshold = 0.05 #0.05
circleRadiusLB = 0.0
circleRadiusUB = 0.1
arg = [fCP, str(threshold), str(circleRadiusLB), str(circleRadiusUB)]
subprocess.call(arg)
print "Done with finding the cutting points"


'''

print "Model the trunk cylinders..."
f = open('cutpoint.txt')
line = f.read().splitlines()
f.close()
if float(line[0]) < 0.06:
    del line[0]
if float(line[0]) < 0.06:
    del line[0]
if float(line[0]) < 0.06:
    del line[0]
print line
line.insert(0, 0.0)
print line[0]
print line[3]
print type(line[0])
#os.system("./locateTrunk/build/findcylinder float(line[0]) float(line[2]) 0 2")
lT = 'locateTrunk/build/findcylinder'
#secStart = float(line
for i in range(0, len(line)-1):
    secStart = float(line[i]) #section starts cutting point
    secEnd = float(line[i+1]) #section ends cutting point
    radiusLB = 0.02 #0.0 #radius lower bound
    radiusUB = 0.075 # radius upper bound
    print "section from " + str(secStart) + "to " + str(secEnd)
    arg = [lT, str(secStart), str(secEnd), str(radiusLB), str(radiusUB)]
    print arg
    subprocess.call(arg)
print "Down with modeling trunk cylinders"



print "Model the branches on each trunk"
tC = 'trunkAndCluster/build/findcylinder'
for i in range(0, len(line)-1):
    secStart = float(line[i]) #section starts cutting point
    secEnd = float(line[i+1]) #section ends cutting point
    outCutBound = 0.03 # cut out boundary
    inCutBound = 0.01 # cut in boundary
    clusterTolerance = 0.015
    minClusterSize = 20
    maxClusterSize = 25000
    disThresholdBranch = 0.05   #0.05
    radiusLB_B = 0.0
    radiusUB_B = 2.0 #2.0
    print "section from " + str(secStart) + "to " + str(secEnd)
    arg = [tC, str(secStart), str(secEnd), str(outCutBound), str(inCutBound), str(clusterTolerance), str(minClusterSize), str(maxClusterSize), str(disThresholdBranch), str(radiusLB_B), str(radiusUB_B)]
    subprocess.call(arg)
print "Done with modeling branch cylinders"


print "Formating result"
print "Starts formating the trunk parameters"
fRT = 'formatResultTrunk/formating'
arg = []
arg.append(fRT)
arg.append(str(len(line)-1))
for i in range(0, len(line)-1):
    if float(line[i]) == 0.0:
        arg.append('cutted_tree_'+'0'+'-'+str(float(line[i+1]))+'.txt')
    else:
        arg.append('cutted_tree_'+str(float(line[i]))+'-'+str(float(line[i+1]))+'.txt') 
print arg
subprocess.call(arg)
print "Done with formating the trunk parameters"





print "Starts formating the branches parameters"
fRB = 'formatResultBranches/formating'
branchRadiusThreshold = 0.05
for i in range(0, len(line)-1):
    path = ''
    arg = []
    arg.append(fRB)
    print "test0"
    if float(line[i]) == 0.0:
	#if os.path.isfile('branchNumber_section'+'0'+'-'+str(line[i+1])+'.txt'):
        path = 'branchNumber_section'+'0'+'-'+str(line[i+1])+'.txt'
            #f = open('branchNumber_section'+'0'+'-'+str(line[i+1])+'.txt')
    else:
        #if os.path.isfile('branchNumber_section'+str(line[i])+'-'+str(line[i+1])+'.txt'):
        path = 'branchNumber_section'+str(line[i])+'-'+str(line[i+1])+'.txt'
            #f = open('branchNumber_section'+str(line[i])+'-'+str(line[i+1])+'.txt')
    if os.path.isfile(path):
        f = open(path)
        branch_num = f.readline()
        f.close()
        arg.append(str(i))
        arg.append(branch_num)
        arg.append(str(line[i]))
        arg.append(str(line[i+1]))
        arg.append(str(branchRadiusThreshold))
        print arg
        subprocess.call(arg)
        print "test1"


