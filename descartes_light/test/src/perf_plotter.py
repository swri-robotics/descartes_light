#!/usr/bin/env python

import math
import numpy as np
# import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization! 
import sys

class Perf_Plotter():
    def __init__(self):
        self.dict = {}

    def strip(self, filename = "../benchmarks/benchmarks.txt"):
        #Each title in "dict" refers to a different algorythm: dict["alg_name"]
        #Each algorithm has a dictionary of lists: dict["alg_name"]['list_name']
        #There are five lists per alg: "dof", "n_waypoints", "samples", "Graph Build Time", "Graph Search Time"
        #Ergo to get each data value, use "dict['alg_name']['key_name'][test#]"
        currentGraphTitle = ' '
        with open(filename) as input:
            for line in input:
                if "descartes_light::" in line:
                    #title lines detected.
                    currentGraphTitle = line[line.find('::')+2:-1]
                    # self.dict[currentGraphTitle] = [] #initialize the value at this key to an empty list
                    self.dict[currentGraphTitle] = {'dofs': [], 'n_waypoints': [], 'samples': [], 'Graph Build Time': [], 'Graph Search Time': []}
                    print("Now reading " + currentGraphTitle)

                elif 'Inputs' in line:
                    #New datapoint for a new simulation: Added as the last dict element for this graph
                    currentDatapointDict = self.stripInputs(line)
                    #add a dictionary to the list of datapoint dictionaries:
                    self.dict[currentGraphTitle]['dofs'].append(float(currentDatapointDict['dofs']))
                    self.dict[currentGraphTitle]['n_waypoints'].append(float(currentDatapointDict['n_waypoints']))
                    self.dict[currentGraphTitle]['samples'].append(float(currentDatapointDict['samples']))
                    

                elif 'Graph' in line:
                    title, value = self.stripTime(line)
                    #Add an entry to the last datapoint dict for this time value:
                    self.dict[currentGraphTitle][title].append(float(value))


    def stripInputs(self, textline):
        #takes inputs in and outputs inputs out.
        workingIndex = textline.find("dof: ") + 5
        out_dict = {}
        out_dict['dofs'] = textline[workingIndex:textline.find(',', workingIndex)]
        workingIndex = textline.find("n_waypoints: ") + 13
        out_dict['n_waypoints'] = textline[workingIndex:textline.find(',', workingIndex)]
        workingIndex = textline.find("samples: ") + 9
        out_dict['samples'] = textline[workingIndex:textline.find('}', workingIndex)]

        return out_dict

    def stripTime(self, textline):
        workingIndex = textline.find(" (s): ")
        title = textline[1:workingIndex]
        workingIndex = workingIndex + 6
        value = textline[workingIndex:-1]
        
        return title, value

    def plot(self, algList, num = 0, which = 0):
        # Format a figure for the specified number "num" of subplots requested; default is all plots in algList

        max_duration = self.find_max(algList[0:num], 'Graph Search Time')
        print("Longest search duration is " + str(max_duration))

        fig = plt.figure()
        plt.subplots_adjust(left=.05, bottom=.1, right=.95, top=.99, wspace=.2, hspace=.15)


        if(num == 1):
            ax = Axes3D(fig)
            alg = algList[which]
            self.subplot(alg, ax)
        else:
            num = min(num, len(algList))
            if num <= 0:
                num = len(algList)
            rows = math.ceil(num/4)
            print("Number of plots is going to be " + str(num) + " using " + str(rows) + " rows.")
            
            for n in range(0, num):
                ax = fig.add_subplot(rows, math.ceil(num/rows), n+1, projection='3d')
                self.subplot(algList[n], ax, max=max_duration)

        plt.show()

    def find_max(self, algs, key):
        _max = 0
        for i in range(0,len(algs)):
            m = np.amax( np.array(self.dict[algs[i]][key]))
            # print("Max in alg " + str(i) + " is " + str(m))
            _max = max(_max, m)
        return _max


    def subplot(self, alg, ax, max = None):
        # Plot alg with average times above clustered time bars. 

        num_tests = len(self.dict[alg][list(self.dict[alg])[0]])
        #print("count of tests = " + str(num_tests))
        test_color_tuples = [ (self.capped_interp(self.dict[alg]['samples'][x], 150, 1),
            self.capped_interp(self.dict[alg]['n_waypoints'][x], 150, 1),
            self.capped_interp(self.dict[alg]['Graph Search Time'][x], 2, 1))
            for x in range(0, num_tests) ]

        y_vals = [self.dict[alg]['n_waypoints'][x]+(.25*x)  for x in range(0, num_tests)  ]
        
        # Generate plot, amazingly just a single command thanks to matplotlib:
        ax.bar(self.dict[alg]['samples'], self.dict[alg]['Graph Search Time'], y_vals, 
             zdir='y', width=12, color=test_color_tuples, alpha=0.75 )

        a = list(set(tuple(zip(self.dict[alg]["samples"], self.dict[alg]["n_waypoints"]))))
        b = zip(self.dict[alg]["samples"], self.dict[alg]["n_waypoints"], self.dict[alg]["Graph Search Time"])
        zs = [0]*len(a)
        count = [0]*len(a)
        #Print a time in seconds for each cluster of data
        for x, y, z in b:
            if (x,y) in a:
                n = a.index((x,y))
                # if(n==3):
                #     print('Index of cluster ' + str(x) + " " + str(y) + " is " + str(n) + " and we're adding " + str(z) + "+"+ str(zs[n]) +"=") 
                zs[n] = (zs[n] + z)
                # if(n==3):
                #     print("Running sum: " + str(zs[n]))
                count[n] += 1.0
            else:
                print("Index Error! Can't find cluster!")
        avg_z = [zs[i]/count[i] for i in range(len(zs))]

        b = zip(a, avg_z)
        for (x, y), z in b:
            if (x,y) in a:
                #If the xy cluster hasn't been labeled yet:
                ax.text(x, y, z+.25, str(np.round(z, 3)), color=(0,0,.8), fontsize=8, zdir='y')
                #del a[a.index(x,y)]
            else:
                print("Index Error! Can't find cluster!")
        # ax.text(10, 10, -.5, "Blah", zdir='y')

        # Format graph
        ax.set_xlabel('Samples')
        ax.set_ylabel('# of Waypoints')
        ax.set_zlabel('Search Time (s)')
        # On the axes let's only label the discrete values that we have data for.
        ax.set_yticks(self.dict[alg]['n_waypoints'])
        ax.set_xticks(self.dict[alg]['samples'])
        if(max != None):
            ax.set_zlim(top = max+.1)
        ax.set_title(alg, fontsize=8)

    def capped_interp(self, input, max, cap):
        return min(input/max, cap)

    def main(self):
        

        self.strip()
        solvers_list = list(self.dict)
        # print("An example algorithm's performance: " + solvers_list[-3])
        # print(str(self.dict[solvers_list[-3]]))
        if len(sys.argv) > 1:
            # argv[1] has your filename
            input1 = sys.argv[1]
            print ("Requested single graph of index " + input1)
            self.plot(solvers_list, 1, int(input1))
        else:
            self.plot(solvers_list, len(solvers_list))
        
if __name__ == '__main__':
    application = Perf_Plotter()
    application.main()