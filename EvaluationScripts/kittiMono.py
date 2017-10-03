#
#	author: Michal Nowicki
#
from subprocess import call
import sys
import os
import fileinput


# Path to save main results - create if needed
if not os.path.exists("results"):
    os.makedirs("results");
else:
    call('rm results/*', shell=True);


sequences = [
    '00', \
    '01', \
    '02', \
    '03', \
    '04', \
    '05', \
    '06', \
    '07', \
    '08', \
    '09', \
    '10', \
    ];

yamls = [
    'KITTI00-02.yaml', \
    'KITTI00-02.yaml', \
    'KITTI00-02.yaml', \
    'KITTI03.yaml', \
    'KITTI04-12.yaml', \
    'KITTI04-12.yaml', \
    'KITTI04-12.yaml', \
    'KITTI04-12.yaml', \
    'KITTI04-12.yaml', \
    'KITTI04-12.yaml', \
    'KITTI04-12.yaml', \
    ];

runsPerSequence = 1;

# mainDatasetPath = '/mnt/data/Datasets/kitti/sequences';
mainDatasetPath = '/media/michalnowicki/MNowicki-Private/KITTI/dataset';

print 'mainDatasetPath: ' + mainDatasetPath

# For all selected sequences
for seq, yaml in zip(sequences, yamls):
    print("Current sequence: " + seq);

    # We call this command
    print('./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/' + str(yaml) + ' ' + str(mainDatasetPath) +'/' + str(seq) +'/');

    # Run code
    call('./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/' + str(yaml) + ' ' + str(mainDatasetPath) +'/' + str(seq) +'/', shell=True);

    # Copy results
    call('mv CameraTrajectory.txt results/' + str(seq) + '.txt', shell=True);
    call('mv KeyFrameTrajectory.txt results/' + str(seq) + '.txt', shell=True);
