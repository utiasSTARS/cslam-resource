%% Load Radius only graphs
clear;
load('data/no_fov_full_runs_rad_downsample_fast_components.mat');
mvc_net = sum(mvc_sol, 5);

%% Plot radius only graphs
% For plotting:
% 1st index is sequence (runs vector), 2nd is radii, 3rd downsamples, 
% 4th fast_thresholds, 5th is components (will be zero for non existent
% ones)
% All the vectors (runs, radii, downsamples, fast_thresholds) are loaded
% with the .mat file

save_path = ['results/radius_results'];

fast_thresh_ind = 1; % 100
% downsample_ind = 3; % 10 (aka 1 Hz)
for downsample_ind=1:length(downsamples)
    for seq_ind=[1,2]
        figure;
        hold on;
        plot(radii, 1e-6*32*squeeze(monolog1_sol(seq_ind,:,downsample_ind, fast_thresh_ind)), 'r-.d', ... 
                                            'markersize', 8,'markerfacecolor','r','linewidth',2);    
        plot(radii, 1e-6*32*squeeze(monolog2_sol(seq_ind,:,downsample_ind, fast_thresh_ind)), '--s', ...
                                            'markersize',8, 'color', [31 117 22]/255, 'markerfacecolor', [31 117 22]/255, 'linewidth',2);
        plot(radii, 1e-6*32*squeeze(mvc_net(seq_ind,:,downsample_ind, fast_thresh_ind)), 'b-o', ...
                                            'markersize',8,'markerfacecolor','b','linewidth',2);
                                        
        v1_min = min(monolog1_sol(seq_ind,:,downsample_ind, fast_thresh_ind));
        v2_min = min(monolog2_sol(seq_ind,:,downsample_ind, fast_thresh_ind));
        mvc_min = min(mvc_net(seq_ind,:,downsample_ind,fast_thresh_ind));
        
        v1_max = max(monolog1_sol(seq_ind,:,downsample_ind, fast_thresh_ind));
        v2_max = max(monolog2_sol(seq_ind,:,downsample_ind, fast_thresh_ind));
        mvc_max = max(mvc_net(seq_ind,:,downsample_ind,fast_thresh_ind));
        min_rad = min(min(v1_min, v2_min), mvc_min);
        max_rad = max(max(v1_max, v2_max), mvc_max);
        ylim([1e-6*32*min_rad, 1e-6*32*max_rad]);
        title_string = ['Seq. ' num2str(runs(seq_ind)) ', Radius'];
        xlabel('max distance (m)','FontSize',20,'interpreter','latex');
        ylabel('communication cost (MB)','FontSize',20,'interpreter','latex');
        set(gca, ...
            'Box'         , 'on'     , ...
            'TickDir'     , 'out'     , ...
            'XMinorTick'  , 'on'     , ...
            'YMinorTick'  , 'on'      , ...
            'YGrid'       , 'on'      , ...
            'XGrid'       , 'on'      , ...
            'XColor'      , [.3 .3 .3], ...
            'YColor'      , [.3 .3 .3], ...
            'LineWidth'   , 1.3         );
        print(gcf, '-depsc', '-r600', [save_path '_seq_' num2str(runs(seq_ind)) '_MB']);
    end
end

%% Load FOV graphs
clear;
% Load in from .mat
load('data/full_runs_overlap_downsample_fast_components.mat');
mvc_net = sum(mvc_sol, 5);

%% Plot overlapping FOV graphs
% For plotting:
% 1st index is sequence (runs vector), 2nd is overlaps, 3rd downsamples, 
% 4th fast_thresholds, 5th is components (will be zero for non existent
% ones)
% All the vectors (runs, radii, downsamples, fast_thresholds) are loaded
% with the .mat file

save_path = ['results/fov_results'];

fast_thresh_ind = 1; % 100
% downsample_ind = 3; % 10 (aka 1 Hz)
for downsample_ind=1:length(downsamples)
    for seq_ind=1:length(runs)
        figure;
        hold on;
        plot(overlaps, 1e-6*32*squeeze(monolog1_sol(seq_ind,:,downsample_ind, fast_thresh_ind)), 'r-.d', ... 
                                            'markersize', 8,'markerfacecolor','r','linewidth',2);    
        plot(overlaps, 1e-6*32*squeeze(monolog2_sol(seq_ind,:,downsample_ind, fast_thresh_ind)), '--s', ...
                                            'markersize',8,'color', [31 117 22]/255, 'markerfacecolor', [31 117 22]/255,'linewidth',2);
        plot(overlaps, 1e-6*32*squeeze(mvc_net(seq_ind,:,downsample_ind, fast_thresh_ind)), 'b-o', ...
                                            'markersize',8,'markerfacecolor','b','linewidth',2);

        title_string = ['Seq. ' num2str(runs(seq_ind)) ', Overlap'];
        xlabel('$\eta$ (FOV overlap fraction)','FontSize',20,'interpreter','latex');
        ylabel('communication cost (MB)','FontSize',20,'interpreter','latex');
        xlim([0.2, 0.8]);
        set(gca, ...
            'Box'         , 'on'     , ...
            'TickDir'     , 'out'     , ...
            'XMinorTick'  , 'on'     , ...
            'YMinorTick'  , 'on'      , ...
            'YGrid'       , 'on'      , ...
            'XGrid'       , 'on'      , ...
            'XColor'      , [.3 .3 .3], ...
            'YColor'      , [.3 .3 .3], ...
            'LineWidth'   , 1.3         );
        print(gcf, '-depsc', '-r600', [save_path '_seq_' num2str(runs(seq_ind))]);
    end
end


%% Load DBoW2 data
clear;
load('data/dbow2_runs_threshold_n_best_downsample_components.mat');
mvc_net = sum(mvc_sol, 5);

%% Plot DBoW2 data
save_path = ['results/dbow2_results'];
fast_thresh_ind = 1;
n_best_ind = 1;
for downsample_ind=1
    for seq_ind=1:length(runs)
        figure;
        hold on;
        plot(threshold_list, 1e-6*32*squeeze(monolog1_sol(seq_ind,:,downsample_ind, fast_thresh_ind)), 'r-.d', ... 
                                            'markersize', 8,'markerfacecolor','r','linewidth',2);    
        plot(threshold_list, 1e-6*32*squeeze(monolog2_sol(seq_ind,:,downsample_ind, fast_thresh_ind)), '--s', ...
                                            'markersize',8,'color', [31 117 22]/255, 'markerfacecolor', [31 117 22]/255,'linewidth',2);
        plot(threshold_list, 1e-6*32*squeeze(mvc_net(seq_ind,:,downsample_ind, fast_thresh_ind)), 'b-o', ...
                                            'markersize',8,'markerfacecolor','b','linewidth',2);

        title_string = ['Seq. ', num2str(runs(seq_ind)) ', DBoW2'];

        
        if runs(seq_ind) == 0
            legend('Monolog 1', 'Monolog 2', 'Optimal');
        end
        xlabel('$\alpha$ (normalized BoW L$_1$ score)','FontSize',20,'interpreter','latex');
        ylabel('communication cost (MB)','FontSize',20,'interpreter','latex');
        set(gca, ...
            'Box'         , 'on'     , ...
            'TickDir'     , 'out'     , ...
            'XMinorTick'  , 'on'     , ...
            'YMinorTick'  , 'on'      , ...
            'YGrid'       , 'on'      , ...
            'XGrid'       , 'on'      , ...
            'XColor'      , [.3 .3 .3], ...
            'YColor'      , [.3 .3 .3], ...
            'LineWidth'   , 1.3         );
        if runs(seq_ind) == 0
            hLegend = legend('Location','NorthEast');
            set(hLegend, 'interpreter', 'latex', 'FontSize', 15);
        end
        
        print(gcf, '-depsc', '-r600', [save_path '_seq_' num2str(runs(seq_ind))]);
    end
end
