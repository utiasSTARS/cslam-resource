%% Min overlap area sweep
clear;
% Order of indices here
runs = [0, 6]; %[0,2,5,6,8];
overlaps = 0.2:0.1:0.8;
downsamples = [1]; %[1,2,5,10,15,20];
fast_thresholds = [100]; %[20, 30, 50, 100, 150, 200];
gurobi_flag = true;

n_comp_max = 100;
mvc_sol = zeros(length(runs), length(overlaps), length(downsamples), length(fast_thresholds), n_comp_max);
monolog_joint_sol = zeros(length(runs), length(overlaps), length(downsamples), length(fast_thresholds));
monolog1_sol = zeros(size(monolog_joint_sol));
monolog2_sol = zeros(size(monolog_joint_sol));
dialogue_dumb_sol = zeros(size(monolog_joint_sol));

tic;
for idx=1:length(runs)
    run = runs(idx);
    run_str = num2str(run);
    if run < 10
        run_str = strcat('0', run_str);
    end    
    pose_string = strcat(strcat('data/odometry/poses_', run_str), '.csv');
    poses = csvread(pose_string);
    for jdx=1:length(overlaps)
        overlap_min = overlaps(jdx);
        for kdx=1:length(downsamples)
            downsample = downsamples(kdx);
            data_string = 'data/odometry/edges_';
            data_string = strcat(data_string, run_str);
            data_string = strcat(data_string, '_r_30_overlap_');
            om_str = num2str(overlap_min);
            om_str = regexprep(om_str, ['\.'], '');
            data_string = strcat(data_string, om_str);
            if downsample ~= 1
                data_string = strcat(data_string, '_downsample_');
                data_string = strcat(data_string, num2str(downsample));
            end
            data_string = strcat(data_string, '.csv');
            E = csvread(data_string);
            E = E+1;
            for ldx=1:length(fast_thresholds)
                w_string = 'data/odometry/weights_';
                w_string = strcat(w_string, run_str);
                w_string = strcat(w_string, '_threshold_');
                w_string = strcat(w_string, num2str(fast_thresholds(ldx)));
                w_string = strcat(w_string, '.csv');
                W = csvread(w_string);
                disp(overlap_min);
                disp(downsample);
                [mvc, ~] = solve_odep(E, W, gurobi_flag);
                mvc_sol(idx,jdx,kdx,ldx, 1:length(mvc)) = mvc;
                mono_1_sol = sum(W(unique(E(:,1))));
                monolog1_sol(idx,jdx,kdx,ldx) = mono_1_sol;
                mono_2_sol = sum(W(unique(E(:,2))));
                monolog2_sol(idx,jdx,kdx,ldx) = mono_2_sol;
                monolog_joint_sol(idx,jdx,kdx,ldx) = min(mono_1_sol, mono_2_sol);
                dialogue_dumb_sol(idx,jdx,kdx,ldx) = mono_1_sol + mono_2_sol;
            end
        end
    end
end
t_taken = toc
save('data/full_runs_overlap_downsample_fast_components.mat');

%% No FOV radius + downsample sweep test
clear;
runs = [0, 6]; %[0,2,5,6,8];
radii = 5:5:30;
downsamples = [1]; %[1,2,5,10,15,20];
fast_thresholds = [100]; %[20, 30, 50, 100, 150, 200];
gurobi_flag = true;

n_comp_max = 100;
mvc_sol = zeros(length(runs), length(radii), length(downsamples), length(fast_thresholds), n_comp_max);
monolog_joint_sol = zeros(length(runs), length(radii), length(downsamples), length(fast_thresholds));
monolog1_sol = zeros(size(monolog_joint_sol));
monolog2_sol = zeros(size(monolog_joint_sol));
dialogue_dumb_sol = zeros(size(monolog_joint_sol));

tic;
for idx=1:length(runs)
    run = runs(idx);
    run_str = num2str(run);
    if run < 10
        run_str = strcat('0', run_str);
    end
    pose_string = strcat(strcat('data/odometry/poses_', run_str), '.csv');
    poses = csvread(pose_string);
    for jdx=1:length(radii)
        rad = radii(jdx);
        for kdx=1:length(downsamples)
            downsample = downsamples(kdx);
            data_string = 'data/odometry/edges_';
            data_string = strcat(data_string, run_str);
            data_string = strcat(data_string, '_r_');
            r_str = num2str(rad);
            r_str = regexprep(r_str, ['\.'], '');
            data_string = strcat(data_string, r_str);
            if downsample ~= 1
                ds_str = num2str(downsample);
                ds_str = strcat('no_fov_downsample_', ds_str);
                data_string = strcat(data_string, ds_str);
            else
                data_string = strcat(data_string, '_no_fov');
            end
            data_string = strcat(data_string, '.csv');
            E = csvread(data_string);
            E = E+1;
            for ldx=1:length(fast_thresholds)
                w_string = 'data/odometry/weights_';
                w_string = strcat(w_string, run_str);
                w_string = strcat(w_string, '_threshold_');
                w_string = strcat(w_string, num2str(fast_thresholds(ldx)));
                w_string = strcat(w_string, '.csv');
                W = csvread(w_string);
                disp(rad);
                [mvc, ~] = solve_odep(E, W, gurobi_flag);
                mvc_sol(idx,jdx,kdx,ldx,1:length(mvc)) = mvc;
                mono_1_sol = sum(W(unique(E(:,1))));
                mono_2_sol = sum(W(unique(E(:,2))));
                monolog1_sol(idx,jdx,kdx,ldx) = mono_1_sol;
                monolog2_sol(idx,jdx,kdx,ldx) = mono_2_sol;
                monolog_joint_sol(idx,jdx,kdx,ldx) = min(mono_1_sol, mono_2_sol);
                dialogue_dumb_sol(idx,jdx,kdx,ldx) = mono_1_sol + mono_2_sol;
            end
        end
    end
end
t_taken = toc;
save('data/no_fov_full_runs_rad_downsample_fast_components.mat');

%% DBoW alpha + n_best sweep
clear;
fast_threshold = 100;
threshold_list = 0.1:0.1:0.9;
n_best_list = [2]; %[1,2,3,4,5,10,20,40,50];
runs = [0, 6]; %[0,2,5,6,8];
n_splits = [2270, 550]; %[2270, 2330, 1380, 550, 2035];
downsamples = [1]; %[1,2,5,10,15,20];
fast_thresholds = [100]; %[20, 30, 50, 100, 150, 200];
n_comp_max = 100;
gurobi_flag = true;

mvc_sol = zeros(length(runs), length(threshold_list), length(n_best_list), length(downsamples), n_comp_max);
monolog_joint_sol = zeros(length(runs), length(threshold_list), length(n_best_list), length(downsamples));
monolog1_sol = zeros(size(monolog_joint_sol));
monolog2_sol = zeros(size(monolog_joint_sol));
dialogue_dumb_sol = zeros(size(monolog_joint_sol));

for idx=1:length(runs)
    run = runs(idx);
    run_str = num2str(run);
    if run < 10
        run_str = strcat('0', run_str);
    end
    w_string = 'data/odometry/weights_';
    w_string = strcat(strcat(w_string, run_str), '_threshold_');
    thresh_string = num2str(fast_threshold);
    w_string = strcat(strcat(w_string, thresh_string), '.csv');
    W = csvread(w_string);
    pose_string = strcat(strcat('data/odometry/poses_', run_str), '.csv');
    poses = csvread(pose_string);
    for jdx=1:length(threshold_list)
        threshold = threshold_list(jdx);
        disp(run);
        disp(threshold);
        for kdx=1:length(n_best_list)
            n_best=n_best_list(kdx);
            for ldx=1:length(downsamples)
                downsample=downsamples(ldx);
                e_string = 'data/odometry/dbow2/edges/edges_dbow2_norm_sequence_';
                e_string = strcat(e_string, run_str);
                e_string = strcat(e_string, '_n_split_');
                e_string = strcat(e_string, num2str(n_splits(idx)));
                e_string = strcat(e_string, '_threshold_');
                thresh_string = num2str(threshold);
                thresh_string = regexprep(thresh_string, ['\.'], '');
                e_string = strcat(e_string, thresh_string);
                e_string = strcat(e_string, '_n_best_');
                e_string = strcat(e_string, num2str(n_best));
                e_string = strcat(e_string, '_downsample_');
                e_string = strcat(e_string, num2str(downsample));
                
                e_string = strcat(e_string, '.csv');
                check_empty = dir(e_string);
                if check_empty.bytes > 0
                    E = csvread(e_string);
                    E = E+1;
                    [mvc, ~] = solve_odep(E, W, gurobi_flag);
                    mvc_sol(idx,jdx,kdx,ldx,1:length(mvc)) = mvc;
                    mono_1_sol = sum(W(unique(E(:,1))));
                    mono_2_sol = sum(W(unique(E(:,2))));
                    monolog1_sol(idx,jdx,kdx,ldx) = mono_1_sol;
                    monolog2_sol(idx,jdx,kdx,ldx) = mono_2_sol;
                    monolog_joint_sol(idx,jdx,kdx,ldx) = min(mono_1_sol, mono_2_sol);
                    dialogue_dumb_sol(idx,jdx,kdx,ldx) = mono_1_sol + mono_2_sol;
                else
                    mvc_sol(idx,jdx,kdx,ldx,:) = NaN;
                    monolog1_sol(idx,jdx,kdx,ldx) = NaN;
                    monolog2_sol(idx,jdx,kdx,ldx) = NaN;
                    monolog_joint_sol(idx,jdx,kdx,ldx) = NaN;
                    dialogue_dumb_sol(idx,jdx,kdx,ldx) = NaN;
                end
            end
        end
    end
end

save('data/dbow2_runs_threshold_n_best_downsample_components.mat');



