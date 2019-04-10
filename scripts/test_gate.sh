#!/bin/bash

DATAPATH=/media/$USER/Elements/AVIS_PhD_Project/Datasets/M3CAM-2.0
SCENARIO=office

vocabulary=ORBvoc.txt

echo $SCENARIO 

VIDEOPATH1=$DATAPATH/$SCENARIO/view1/rgb/
VIDEOPATH2=$DATAPATH/$SCENARIO/view2/rgb/
VIDEOPATH3=$DATAPATH/$SCENARIO/view3/rgb/

./demo_orb $VIDEOPATH1 $VIDEOPATH2 $vocabulary
mv scores.dat scores_v1_v2_$SCENARIO.dat

