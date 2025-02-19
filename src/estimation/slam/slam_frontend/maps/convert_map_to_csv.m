%
% AMZ Driverless Project
%
% Copyright (c) 2023-2024 Authors:
%   - Christoforos Nicolaou <cnicolaou@ethz.ch>
%
% All rights reserved.
%
% Unauthorized copying of this file, via any medium is strictly prohibited
% Proprietary and confidential
%

% TODO(Christoforos) Use python script instead

clear all

% Load the ROS bag file
bag = rosbag('skidpad.bag');

% Select the topic with the ConeArray message
topic = select(bag, 'Topic', '/estimation/global_map');

% Read all messages from the selected topic.
% There is only one message.
msgs = readMessages(topic, 'DataFormat', 'struct');

% Extracting data from the ConeArray message
coneArray = msgs{1};

% Prepare data for CSV
csvData = {};
for i = 1 : length(coneArray.Cones)
    cone = coneArray.Cones(i);
    csvRow = {cone.Position.X, cone.Position.Y, cone.ProbCone, cone.ProbType.Blue, cone.ProbType.Yellow, cone.ProbType.Orange, cone.ProbType.OrangeBig};
    csvData = [csvData; csvRow];
end

% Define CSV file name
csvFileName = 'skidpad.csv';

% Write to CSV
writecell(csvData, csvFileName);
