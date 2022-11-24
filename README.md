# README #

Used for making plots of walking speed from the short walk IMU experiment, where people walked 10 different short distances. All data are stored in saved_walk_info. 

To use, run analyze_short_walks_new.m. Ignore optimization warnings

This cannot be used for any generalized data without modification because there was a lot of manual intervention in finding the intersecting points in the position-time graph to set step speed and time. These interventions have been hard-coded for specific subjects and trials and would have to be removed in order to perform the same analysis on a new data set.

The m-file performs analysis and makes plots for each subject, and then
produces a summary plot for all subjects. Each subject walked 10 distances (conditions),
repeated for multiple trials. There were two terrains, sidewalk ("flat") and
grass ("uneven"). 

The plots for each subject include all trials
* Forward position vs time for the two feet (alternating lines)
* Step speed vs. time
* Peak speed vs distance (including a curve fit)
* Duration vs distance (including a curve fit)
The curve fits are explained in the manuscript, and are used only to describe
the general trend in the data. The plots are also saved as image files.

The optimization warnings come from an optimization to determine the best
scaling factor between IMUs. The integrated IMU displacements do not match
exactly, and so the optimization determines the scaling factor so that
both IMUs walk about the same distance. Warnings are issued when the
optimization finds a local minimum. The warnings may safely be ignored, 
because poor scaling mainly causes the speeds to be jagged--the alternating
speeds of the two feet will go up and down during steady walking.

