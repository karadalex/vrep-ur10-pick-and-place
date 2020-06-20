figure('Name', 'Position-Velocity diagrams in joint space (Exported VREP Path)');
% initialize vrep_q_data
vrep_q_data = csvread("./vrep/graphs/q1.csv", 2, 0);
vrep_data_rows = size(vrep_q_data,1);
vrep_q_data = zeros(vrep_data_rows, 3, 6);
for i=1:6
    vrep_q_data(:,:,i) = csvread("./vrep/graphs/q"+i+".csv", 2, 0);
    subplot(3,2,i);
    plot(vrep_q_data(:,1,i), vrep_q_data(:,2,i), 'b', vrep_q_data(:,1,i), vrep_q_data(:,3,i), 'g');
    title("q"+i);
    xlabel("steps");
    ylabel("rads, rads/s"); 
end

figure('Name', 'Position-Velocity diagrams in task space (Exported VREP Path)');
% initialize vrep_r_data
vrep_r_data = zeros(vrep_data_rows, 3, 6);
r_labels = ["x", "y", "z", "roll", "pitch", "yaw"];
for i=1:6
    r = r_labels(i);
    vrep_r_data(:,:,i) = csvread("./vrep/graphs/"+r+".csv", 2, 0);
    subplot(3,2,i);
    plot(vrep_r_data(:,1,i), vrep_r_data(:,2,i), 'b', vrep_r_data(:,1,i), vrep_r_data(:,3,i), 'g');
    title(r);
    xlabel("time (seconds)");
    if i<4
        ylabel("meters, meters/s"); 
    else
        ylabel("rads, rads/s");
    end
end