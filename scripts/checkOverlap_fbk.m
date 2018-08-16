%    Creation Date: 2018/07/03
%    Modified Date: 2018/07/03
%           Author: Alessio Xompero
%            email: a.xompero@qmul.ac.uk
%--prologue
clear all; close all; clc;

fs = 20;

datasets = {'office', 'desk' , 'courtyard', 'kitti', 'fbk'};



% C = [1 1 2
%   2 1 2
%   3 1 2
%   4 1 2];

C = [5 1 2
  5 1 3
  5 1 4
  5 1 5];

ss = get(0,'ScreenSize');
figure('Position',[0 0 ss(3) ss(4)])

for ssd=1:size(C,1)
  d = C(ssd,1);
  v1 = C(ssd,2);
  v2 = C(ssd,3);
  
  dataset=datasets{d};
  
  disp([dataset ' dataset: image #' num2str(v1) ' vs image #' num2str(v2)])
  
  DATAPATH=fullfile('res',dataset, ['res_' dataset '_' num2str(v1) num2str(v2)]);
  
  %%% Load DBoW2 scores
  S = dlmread(fullfile(DATAPATH, ['scores_' num2str(v1) num2str(v2) '_' dataset '.dat']));
  M = dlmread(fullfile(DATAPATH, 'matches.dat'));
  N = dlmread(fullfile(DATAPATH, 'matches_normalised.dat'));
  
  %%% Check if matrix is symmetric
%   if issymmetric(S) == 0
%     disp('Matrix S is not symmetric')
%   end
%   
%   if issymmetric(M) == 0
%     disp('Matrix M is not symmetric')
%   end
%   
%   if issymmetric(N) == 0
%     disp('Matrix N is not symmetric')
%   end
  
  %%%
  subplot(2,2,1)
  imagesc(S)
  axis square
  
  ch = colorbar;
  caxis([0 1])
  ch.Label.String = 'DBoW2 score';
  
  ylabel('Frame index (Camera 1)')
  
  
  title(dataset,'Interpreter','none')
  
  set(gca,'FontSize',fs)
  
  %%%
  subplot(2,2,2)
  imagesc(S)
  axis square
  
  ch = colorbar;
  ch.Label.String = 'DBoW2 score';
  
  ylabel('Frame index (Camera 1)')
  %   xlabel('Frame index (Camera 2)')
  
  set(gca,'FontSize',fs)
  
  %     %%%
  %   subplot(2,2,3)
  %   imagesc(M)
  %   axis square
  %
  %   ch = colorbar;
  %   ch.Label.String = '# matches';
  %
  %   ylabel('Frame index (Camera 1)')
  %   xlabel('Frame index (Camera 2)')
  %
  %   set(gca,'FontSize',fs)
  
  %%%
  subplot(2,2,4)
  imagesc(N)
  axis square
  
  ch = colorbar;
  ch.Label.String = 'Matching score';
  
  ylabel('Frame index (Camera 1)')
  xlabel('Frame index (Camera 2)')
  
  title('# ORB features = 2000')
  
  set(gca,'FontSize',fs)
  
  
  %-----
  print(fullfile(DATAPATH, ['dbow2_score_' num2str(v1) num2str(v2) '_' dataset]),'-dpng')
  %   print(fullfile(DATAPATH, ['dbow2_score_' num2str(v1) num2str(v2) '_' dataset]),'-depsc2')
  
  pause(0.1)
  
  clf
  
  [m,n] = find(S == max(S(:)),1);
  disp(num2str(length(m)))
  disp(['Best image pair (sim score) in ' dataset ': (' num2str(m) ',' num2str(n) ') -> ' num2str(S(m,n)) ' (' num2str(M(m,n)) ' matches)'])
  
  [m,n] = find(N == max(N(:)),1);
  disp(['Best image pair (# matches) in ' dataset ': (' num2str(m) ',' num2str(n) ') -> ' num2str(S(m,n)) ' (' num2str(M(m,n)) ' matches)'])
  disp('\n')
end

close all

disp('Finished')
