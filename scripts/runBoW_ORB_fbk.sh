#!/bin/bash


#dataset=coslam_courtyard_2
dataset=fbk
ID1=1
ID2=5

vocabulary=ORBvoc.txt

echo $dataset 

DATAPATH=/home/alessioxompero/Desktop/AVIS_PhD_Project/Datasets/M3CAM

#videopath1=/home/alessioxompero/Desktop/AVIS_PhD_Project/Datasets/TORB/$dataset/view1/rgb/
#videopath2=/home/alessioxompero/Desktop/AVIS_PhD_Project/Datasets/TORB/$dataset/view2/rgb/


videopath1=$DATAPATH/$dataset/view$ID1/rgb/
videopath2=$DATAPATH/$dataset/view$ID2/rgb/

./demo_orb   $videopath1 $videopath2 $vocabulary
mv scores.dat scores_${ID1}${ID2}_$dataset.dat

tar -czvf res_${dataset}_${ID1}${ID2}.tar.gz matches.dat matches_normalised.dat scores_${ID1}${ID2}_${dataset}.dat

rm matches.dat matches_normalised.dat scores_${ID1}${ID2}_${dataset}.dat
