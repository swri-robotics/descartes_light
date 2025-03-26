#!/usr/bin/env python
import argparse
import pathlib
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization! 
import sys

class PerfPlotter():
    def __init__(self):
        self.dict = {}

    def strip(self, filename):
        """Generates a dictionary of algorithm benchmark data. Output stored inside Perf_Plotter.dict.
        Data values are organized in the form "Perf_Plotter.dict['algorithm_name'(string)]['data_key_name'(string)][test#(int)]"
        :param filename: (string) Relative path to text file containing benchmark data of interest.
        """
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
                    currentDatapointDict = self.strip_inputs(line)
                    #add a dictionary to the list of datapoint dictionaries:
                    self.dict[currentGraphTitle]['dofs'].append(float(currentDatapointDict['dofs']))
                    self.dict[currentGraphTitle]['n_waypoints'].append(float(currentDatapointDict['n_waypoints']))
                    self.dict[currentGraphTitle]['samples'].append(float(currentDatapointDict['samples']))
                    

                elif 'Graph' in line:
                    title, value = self.strip_time(line)
                    #Add an entry to the last datapoint dict for this time value:
                    self.dict[currentGraphTitle][title].append(float(value))

    def strip_inputs(self, textline):
        """Scrapes test config data from a line in a benchmarks file.
        :param textline: (string) Line of text from which to strip data according to text delimeters.
        :return: (dict) Label and value pairs of data in input.
        """
        workingIndex = textline.find("dof: ") + 5
        out_dict = {}
        out_dict['dofs'] = textline[workingIndex:textline.find(',', workingIndex)]
        workingIndex = textline.find("n_waypoints: ") + 13
        out_dict['n_waypoints'] = textline[workingIndex:textline.find(',', workingIndex)]
        workingIndex = textline.find("samples: ") + 9
        out_dict['samples'] = textline[workingIndex:textline.find('}', workingIndex)]

        return out_dict

    def strip_time(self, textline):
        """Scrapes time data and associated label from a line in a benchmarks file.
        :param textline: (string) Line of text from which to strip time and label.
        :return: (tuple of string, float) Label and value of time data in input.
        """
        workingIndex = textline.find(" (s): ")
        title = textline[1:workingIndex]
        workingIndex = workingIndex + 6
        value = textline[workingIndex:-1]
        
        return title, value

    def plot(self, algList):
        """Create a figure and plot the indicated algorithm performances in subplots.
        :param algList: (list) A list of names (String) of algorithms to plot.
        """
        # print("Alg list is " + str(algList))
        num=len(algList)
        max_duration = self.find_max(algList[0:num], 'Graph Search Time')
        # print("Longest search duration is " + str(max_duration))

        fig = plt.figure()
        plt.subplots_adjust(left=.05, bottom=.1, right=.95, top=.99, wspace=.2, hspace=.15)

        if num <= 0:
            print("Error: No elements to graph!")
            num = len(algList)
        rows = math.ceil(num/4)
        print("Number of plots = " + str(num) + 
            " using " + str(rows) + " rows.")
            
        for n in range(0, num):
            ax = fig.add_subplot(rows, math.ceil(num/rows), n+1, projection='3d')
            self.subplot(algList[n], ax, max=max_duration)

        plt.show()

    def find_max(self, algs, key):
        """Scans through performance data to find max value for a metric (time, #samples, etc.)
        :param algs: (list) A list of names (String) of algorithms to check.
        :param key: (String) The metric type to plot (must be "dofs", "n_waypoints", "samples", "Graph Search Time", or "Graph Build Time")
        """
        _max = 0
        for i in range(0,len(algs)):
            m = np.amax( np.array(self.dict[algs[i]][key]))
            # print("Max in alg " + str(i) + " is " + str(m))
            _max = max(_max, m)
        return _max

    def subplot(self, alg, ax, max = None):
        """ Plot alg with average times above clustered time bars.
        :param alg: (dict) Dictionary of lists of per-test data addressed by keys 'samples', 
         'n_waypoints', and 'Graph Search Time'.
        :param ax: (mpl_toolkits.mplot3d.axes3d) 3d plot axes object onto which to plot data.
        :param max: (float) Optional maximum z-axis value; by default max is greater than
         maximum z value of plotted data.
        """ 

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

        #Print the average time in seconds for each cluster of data
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
                ax.text(x, y, z+.1, str(np.round(z, 3)), color=(0,0,.8), fontsize=8, zdir='y')
            else:
                print("Index Error! Can't find cluster!")

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

def main():
    """Parse inputs, parse input data file; select graphs to display; show them. 
    """
    parser = argparse.ArgumentParser(description='Display graphs based on benchmark outputs. ' +
        'Numbers above tests are the average search time for all tests with identical parameters.')
    parser.add_argument('--graphs', metavar='N', type=int, nargs='+', default="0",
        help='Specific tests to graph (first graph = 1). If not included, graph all tests in file.')
    parser.add_argument('--datapath', type=pathlib.Path, default="../benchmarks/benchmarks.txt",
        help='File to process (default: ../benchmarks/benchmarks.txt')

    args = parser.parse_args()
    application = Perf_Plotter()
    application.strip(args.datapath)

    if(args.graphs):
        # If graph numbers give, plot those; otherwise, plot all.
        #For simplicity, list the titles of all algorithms found in benchmark file
        algorithms_list = list(application.dict)

        # Establish a list of graph indices to actually graph
        to_graph = list()
        
        for i in args.graphs:
            try:
                to_graph.append(algorithms_list[i-1])
            except IndexError:
                print("Error: Index " + str(i) + " invalid for list of indices 0 - " + str(len(list(application.dict))-1))
            
    else:
        to_graph = list(application.dict)
    application.plot(to_graph)

if __name__ == '__main__':
    main()
