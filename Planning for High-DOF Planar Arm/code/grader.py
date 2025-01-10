import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

plannerList = ["RRT", "RRTSTAR", "PRM"]

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV):
    problems = [["./map2.txt", "1.7,0.76,1.5,2.6,0.33",
                                "0.89,2.7,0.36,4.71.1.9"],
            ["./map2.txt", "1.7,1,0.3,5.6,4.2",
                                "1.8,0.2,1.5,2.6,5.2"],
            ["./map2.txt", "1.4,6.0,2.3,2.2,1.1",
                                "1.0,1.0,2.3,4.5,2.7"],
            ["./map2.txt", "1.5,1.9,3.1,1.3,3.9",
                                "1.6,0.9,0,5.2,1.5"],
            ["./map2.txt", "1.6,2.1,2.15,5.8,0.34",
                                "0.44,2.66,1.68,3.23,2.89"],
            ["./map2.txt", "1.3,0.7,2.8,1.2,3",
                                "1.7,2.3,4,4.1,1.8"],
            ["./map2.txt", "1.0,2.0,0.19,1.47,4.2",
                                "1.58,3.1,3.0,0.36,0.96"],
            ["./map2.txt", "1.1,2.69,5.58,2.5,0.72",
                                "1.29,1.39,4.76,5.77,0.29"],
            ["./map2.txt", "1.1,0.9,2,1.9,1.1",
                                "1.7,6.2,1.7,0.4,0.6"],
            ["./map2.txt", "1.1,2.7,0.11,0.29,5.79",
                                "0.69,2.2,1.83,1.0,5.62"],
            ["./map2.txt", "1.64,1.1,3.14,2.36,3.67",
                                "1.7,2.47,0.8,0.98,1.68"],
            ["./map2.txt", "0.43,1.8,1.78,3.84,2.1",
                                "1.36,0.86,4.55,1.33,3.7"],
            ["./map2.txt", "1.5,1.7,5.9,0.1,4.9",
                                "1.8,2.5,1.8,1.4,4.3"],
            ["./map2.txt", "1.5,2.7,5.5,1.69,0.32",
                                "0.31,2.26,1.54,6.27,2.68"],
            ["./map2.txt", "1.5,1.3,2.2,1.15,0.83",
                                "1.69,2.2,5.2,1.0,2.99"],
            ["./map2.txt", "1.1,2.6,5.5,2.5,0.72",
                                "1.3,1.39,4.7,5.7,0.3"],
            ["./map2.txt", "0.11,0.48,3.33,2.6,0.78",
                                "1.35,1.68,3.1,4.68,2.7"],
            ["./map2.txt", "1.7,0.09,4.3,1.23,5.16",
                                "0.84,2.7,0.39,2.3,0.42"],
            ["./map2.txt", "0.99,1.7,0.4,3.72,5.46",
                                "1.0,1.0,2.3,5.6,0.82"],
            ["./map2.txt", "0.39,1.59,2.89,0.69,0.14",
                                "0.6,2.95,1.6,0.9,2.0"]]
    scores = []
    for aPlanner in [0, 1, 2]:
        print("\nTESTING " + plannerList[aPlanner] + "\n")
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "grader_out/tmp.txt"
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPos, goalPos,
                aPlanner, outputSolutionFile)
            print("EXECUTING: " + str(commandPlan))
            commandVerify = "./verifier.out {} {} {} {} {}".format(
                inputMap, numDOFs, startPos, goalPos,
                outputSolutionFile)
            print("EXECUTING: " + str(commandVerify))
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                ### Visualize their results
                commandViz = "python visualizer.py grader_out/tmp.txt --gifFilepath=grader_out/grader_{}{}.gif".format(plannerList[aPlanner], i)
                commandViz += " --incPrev=1"
                subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./planner.out", "grader_out/grader_results.csv")