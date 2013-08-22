#!/usr/bin/env python
###############################################################################
#                                                                             #
#   PixArt Data Log Dumper                                                    #
#   2012 Andrew Beckett - drasnor@tamu.edu                                    #
#                                                                             #
#   This program parses the sample data logs generated by the PixArt logging  #
#   firmwares and writes the results to a .csv file for use in other software #
#   analysis packages.                                                        #
#                                                                             #
###############################################################################

# Modules
import argparse
import csv
import pixartLogData

# Defaults
defaultOutfile = "samples.csv"

## Parse command line arguments
parser = argparse.ArgumentParser(prog='Glider Log Dumper',
                                 description='Parses glider binary data logs')
parser.add_argument('infile', help='path and filename of the binary sample file')
parser.add_argument('-o',
                    dest='outfile',
                    help='path and filename of the output csv file'
)
args = parser.parse_args()

## Read samples binary file
sampleFile = open(args.infile,'rb')
dataString = sampleFile.read()

## Parse stream
data = gliderData.experiment(dataString)

## Generate report
if args.outfile:
    csvOut = csv.writer(open(args.outfile,'w'),lineterminator='\n')
else: csvOut = csv.writer(open(defaultOutfile,'w'),lineterminator='\n')

# Write each row
for i in xrange(len(data.sessions)): # for each session
    # Write a header
    csvOut.writerow(["timestamp (ms)","blobMask",
                     "Blob0 X","Blob0 Y",
                     "Blob1 X","Blob1 Y",
                     "Blob2 X","Blob2 Y",
                     "Blob3 X","Blob3 Y"])
    for key in sorted(data.sessions[i].blobs.iterkeys()): # for each time instant
        timestamp = key
        blobMask = data.sessions[i].blobs[key]["blobMask"]
        if "Blob0" in data.sessions[i].blobs[key]:
            Blob0X = data.sessions[i].blobs[key]["Blob0"]['X']
            Blob0Y = data.sessions[i].blobs[key]["Blob0"]['Y']
        else:
            Blob0X = None
            Blob0Y = None
        if "Blob1" in data.sessions[i].blobs[key]:
            Blob1X = data.sessions[i].blobs[key]["Blob1"]['X']
            Blob1Y = data.sessions[i].blobs[key]["Blob1"]['Y']
        else:
            Blob1X = None
            Blob1Y = None
        if "Blob2" in data.sessions[i].blobs[key]:
            Blob2X = data.sessions[i].blobs[key]["Blob2"]['X']
            Blob2Y = data.sessions[i].blobs[key]["Blob2"]['Y']
        else:
            Blob2X = None
            Blob2Y = None
        if "Blob3" in data.sessions[i].blobs[key]:
            Blob3X = data.sessions[i].blobs[key]["Blob3"]['X']
            Blob3Y = data.sessions[i].blobs[key]["Blob3"]['Y']
        else:
            Blob3X = None
            Blob3Y = None
        csvOut.writerow([timestamp,blobMask,
                         Blob0X,Blob0Y,
                         Blob1X,Blob1Y,
                         Blob2X,Blob2Y,
                         Blob3X,Blob3Y])

