
import csv
import sys

#defining the track width
# opening the file
with open('/home/jet/Downloads/trajectory_generator/outputs/wp/traj_race_cl.csv', mode ='r')as file:
   
    # reading the CSV file
    csvFile = csv.reader(file)
    x = []
    y = []
    vel = []
    i=0
    for lines in csvFile:
        i+=1
        if i<=3:
            continue
        else:
            x.append(lines[1])
            y.append(lines[2])
            vel.append(lines[5])
#remove first element
x = x[1:]
y = y[1:]
# print(len(x))
# write to file
with open('/home/jet/Downloads/trajectory_generator/final_raceline.csv', mode='w') as csv_file:
    #write to csv
    csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for i in range(len(x)):
        csv_writer.writerow([x[i], y[i], vel[i]])


