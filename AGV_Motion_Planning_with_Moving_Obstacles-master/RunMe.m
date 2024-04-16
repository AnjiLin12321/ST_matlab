% ==============================================================================
% % Source Codes for "Fast Trajectory Planning for AGV in the Presence of
% Moving Obstacles: A Combination of 3-dim A* Search and QCQP".
% Bai Li, Youmin Zhang, Yi Liu, Xiang Zhong, Hangjie Cen, Xiaoyan Peng, Qi Kong
% 33rd Chinese Control and Decision Conference (CCDC), accepted on Feb. 15, 2021.
% ==============================================================================
% Copyright (C) 2021 Bai Li. Users MUST cite the related articles
% mentioned in http://grjl.hnu.edu.cn/p/2019256
% License GNU General Public License v3.0
% Should request a licensed AMPL with IPOPT. Put the binary files into the
% current folder.
% ==============================================================================
clear all; close all; clc;

global params_
%障碍物信息 
params_.radius = 2;
params_.Nobs = 5;
params_.Nfe = params_.NT * 3;  %80*3
params_.obs = GenerateDynamicObstacles();  % 5个障碍物start和end points 随机生成

%地图信息 地图大小
params_.x_min = -20;
params_.x_max = 20;
params_.y_min = -20;
params_.y_max = 20;
params_.x_scale = params_.x_max - params_.x_min;
params_.y_scale = params_.y_max - params_.y_min;
params_.NX = 20;
params_.NY = 20;
params_.dx = params_.x_scale / (params_.NX - 1);
params_.dy = params_.y_scale / (params_.NY - 1);

%预测步0.5s
params_.tf_max = 40;
params_.NT = 80;
params_.dt = params_.tf_max / (params_.NT - 1);

%求解参数
params_.weight_for_time = 100.0;
params_.Nring = 1;
params_.max_iter = 2000;

%小车起始点终点 随机
params_.x0 = params_.x_min + params_.radius + (params_.x_scale - 2 * params_.radius) * rand;
params_.y0 = params_.y_min + params_.radius + (params_.y_scale - 2 * params_.radius) * rand;
params_.xf = params_.x_min + params_.radius + (params_.x_scale - 2 * params_.radius) * rand;
params_.yf = params_.y_min + params_.radius + (params_.y_scale - 2 * params_.radius) * rand;



[x, y] = SearchTrajViaAstar();
if (length(x) < params_.NT)
    error 1;
end

[x, y, ready_flag] = RefinePathViaNLP(x, y);
DemonstrateDynamicResult(x, y);