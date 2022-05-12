# README #

Used for making plots of walking speed from the short walk IMU experiment, where people walked 10 different short distances. All data are stored in saved_walk_info. 

To use, run analyze_short_walks_new.m

This cannot be used for any generalized data without modification because there was a lot of manual intervention in finding the intersecting points in the position-time graph to set step speed and time. These interventions have been hard-coded for specific subjects and trials and would have to be removed in order to perform the same analysis on a new data set.