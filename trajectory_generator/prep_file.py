#!/usr/bin/python

import csv
import sys

#defining the track width
track_width = sys.argv[1] 

# opening the file
with open('path.csv', mode ='r')as file:
   
    # reading the CSV file
    csvFile = csv.reader(file)
    x = []
    y = []
    for lines in csvFile:
        x.append(lines[0])
        y.append(lines[1])

#remove first element
x = x[1:]
y = y[1:]
#write to file
with open('/trajectory_generator/inputs/tracks/wp', mode='w') as csv_file:
    #write to csv
    csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for i in range(len(x)):
        csv_writer.writerow([x[i], y[i], track_width, track_width])


