%    Creation Date: 2018/07/03
%    Modified Date: 2018/07/03
%           Author: Alessio Xompero
%            email: a.xompero@qmul.ac.uk
%--prologue
clear all; close all; clc;

datasets = {'freiburg_office', 'freiburg_desk' , 'coslam_courtyard_2', ...
  'kitti', 'fbk_outdoor'};

C = [1 1 2
  2 1 2
  3 1 2
  4 1 2];

for ssd=1:size(C,1)
  d = C(ssd,1);
  v1 = C(ssd,2);
  v2 = C(ssd,3);
  
  dataset=datasets{d};
  
  %%% Load DBoW2 scores
  S = dlmread(['scores_' num2str(v1) num2str(v2) '_' dataset '.dat']);
  
  [m,n] = find(S == max(S(:)),1);
  
  disp([dataset ': (' num2str(m) ',' num2str(n) ') -> ' num2str(max(S(:)))])
end

close all
disp('Finished')
