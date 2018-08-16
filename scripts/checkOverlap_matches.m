%    Creation Date: 2018/07/03
%    Modified Date: 2018/07/03
%           Author: Alessio Xompero
%            email: a.xompero@qmul.ac.uk
%--prologue
clear all; close all; clc;

datasets = {'freiburg_office', 'freiburg_desk' , 'coslam_courtyard_2', ...
  'kitti', 'fbk_outdoor'};

% datasets = {'office', 'desk' , 'courtyard', 'kitti', 'fbk'};

% C = [1 1 2
%   2 1 2
%   3 1 2
%   4 1 2];

C = [3 1 2];

for ssd=1:size(C,1)
  d = C(ssd,1);
  v1 = C(ssd,2);
  v2 = C(ssd,3);
  
  dataset=datasets{d};
  %%% Load DBoW2 scores
%   S = dlmread(['scores_' num2str(v1) num2str(v2) '_' dataset '.dat']);
  S = dlmread('matches.dat');
  
  %%% Check if matrix is symmetric
  if issymmetric(S) == 0
    disp('Matris is not symmetric')
  end
  
  ss = get(0,'ScreenSize');
  
  imagesc(S)
  axis square
  
  ch = colorbar;
%   caxis([0 1])
  ch.Label.String = '# of ORB matches';
  
  ylabel('Frames of first video stream')
  xlabel('Frames of second video stream')
  
  title(dataset,'Interpreter','none')
  
  set(gca,'FontSize',20)
  set(gcf,'Position',[0 0 ss(3) ss(4)])
  
  print(['dbow2_matches_' num2str(v1) num2str(v2) '_' dataset],'-dpng')
%   print(['dbow2_matches_' num2str(v1) num2str(v2) '_' dataset],'-depsc2')
  
  pause(0.1)
  
  clf
end

close all



%%%%
for ssd=1:size(C,1)
  d = C(ssd,1);
  v1 = C(ssd,2);
  v2 = C(ssd,3);
  
  dataset=datasets{d};
  %%% Load DBoW2 scores
%   S = dlmread(['scores_' num2str(v1) num2str(v2) '_' dataset '.dat']);
  S = dlmread('matches_normalised.dat');
  
  %%% Check if matrix is symmetric
  if issymmetric(S) == 0
    disp('Matrix is not symmetric')
  end
  
  ss = get(0,'ScreenSize');
  
  imagesc(S)
  axis square
  
  ch = colorbar;
%   caxis([0 1])
  ch.Label.String = 'Matching score';
  
  ylabel('Frames of first video stream')
  xlabel('Frames of second video stream')
  
  title(dataset,'Interpreter','none')
  
  set(gca,'FontSize',20)
  set(gcf,'Position',[0 0 ss(3) ss(4)])
  
  print(['dbow2_matchesnorm_' num2str(v1) num2str(v2) '_' dataset],'-dpng')
%   print(['dbow2_matches_' num2str(v1) num2str(v2) '_' dataset],'-depsc2')
  
  pause(0.1)
  
  clf
end

close all

%--------------------
disp('Finished')
