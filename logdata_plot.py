import matplotlib.pyplot as plt
import numpy as np
import struct as st
import os.path
import csv
import sys

logdata = []
label = ['COUNT_time', 'drivinstage', 'turn',
         'omega', 'hsv.val', 'distance', 'gyro_deg']
colorlist = ['black', 'tab:red', 'tab:green', 'tab:blue',
             'tab:cyan', 'tab:olive', 'tab:purple',
             'tab:orange', 'tab:brown', 'tab:gray']

# for native linux
#logdatfile = os.path.expanduser('~/ramdisk/log.dat')
#logcsvfile = os.path.expanduser('~/ramdisk/log.csv')
# for windows
logdatfile = os.path.join(os.path.dirname(__file__), 'log.dat')
logcsvfile = os.path.join(os.path.dirname(__file__), 'log.csv')

with open(logdatfile, 'rb') as logfile:
    size = st.calcsize('Iiiiiii')
    content = logfile.read(size)
    while content:
        decode_data = st.unpack('Iiiiiii', content)
        logdata.append(decode_data)
        content = logfile.read(size)

with open(logcsvfile, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile, dialect="excel")
    writer.writerow(label)
    writer.writerows(logdata)
'''
writer = csv.writer(sys.stdout, dialect="excel")
writer.writerow(label)
writer.writerows(logdata)
'''

arr_log = np.array(logdata).T.tolist()
# print(np.shares_memory(logdata, arr_log))

# graph
fig = plt.figure(figsize=(18, 7))
ax = fig.add_subplot()

for i in range(1, len(arr_log)):
    ax.plot(arr_log[0], arr_log[i], label=label[i], marker=".")
    #ax.scatter(arr_log[0], arr_log[i], label=label[i])

ax.set_xlabel('time[ms]')
ax.set_ylabel('PID value')
ax.grid()
ax.legend(loc='best')
fig.tight_layout()
plt.show()
