clear variables;
data = readmatrix('trade.csv','range','B2:H6');
data_norm = zeros(size(data,1),size(data,2));

%normalize data to be between 0 and 1 for each property
for i=1:size(data,1)
   for j=1:size(data,2)
      data_norm(i,j) = (data(i,j) - min(data(i,:))) / (max(data(i,:)) - min(data(i,:)));
   end
end

data_norm(1,:) = 1*data_norm(1,:); %change c_D(0) weighting
data_norm(2,:) = 1*data_norm(2,:); %change c_D'' weighting
data_norm(3,:) = 0.7*data_norm(3,:); %change max cM weighting
data_norm(4,:) = 1*data_norm(4,:); %change c_L' weighting
data_norm(5,:) = 0.5*data_norm(5,:); %change effective range weighting

%adjust normalization to be between 1 and 2 to avoid 0 in calculations
data_norm = data_norm + 1;

scores = zeros(1,size(data_norm,2));
%assign scores
for j=1:size(data_norm,2)
   %score = (c_L' * range) / (c_D(0) * c_D'' * max_c_M)
   scores(j) = data_norm(4,j)*data_norm(5,j) / (data_norm(1,j)*data_norm(2,j)*data_norm(3,j));
end

disp("    0024      0018      0015      0012      0010      0008      0006");
disp(scores);
