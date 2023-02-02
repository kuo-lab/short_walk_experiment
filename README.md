# README short_walk_experiment #

## [Optimization of energy and time predicts dynamic speeds for human walking (Carlisle and Kuo 2022)](https://github.com/kuo-lab/simplelocomotionmodel)

This repository contains code for making plots of walking speed from the short walk IMU experiment, where people walked 10 different short distances. All data are stored in `saved_walk_info/new`. 

To use, run `analyze_short_walks_new.m`. Ignore optimization warnings

The data analysis included significant manual intervention for identifying the intersecting points in the position-time graph to set step speed and time. These interventions have been hard-coded for specific subjects and trials and would have to be removed in order to perform the same analysis on a new data set. Otherwise, much of the analysis could be applied to other data for short walking bouts. 

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

## Optimization model
The code for the dynamic optimization model is in a [separate optimization repository](https://github.com/kuo-lab/simplelocomotionmodel). The code is in Julia and the entire modeling and optimization toolchain is available as open source.

# References
<div id="refs" class="references csl-bib-body hanging-indent">

<div id="ref-carlisle2022OptimizationEnergyTime" class="csl-entry">

Carlisle, R. Elizabeth, and Arthur D. Kuo. 2022. “Optimization of Energy
and Time Predicts Dynamic Speeds for Human Walking.” bioRxiv.
<https://doi.org/10.1101/2022.07.15.500158>.