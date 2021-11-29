map = imread('demo2.png');
map = rgb2gray(map);
map = imrotate(map,-90);
map = flip(map,2);

map = 255 - map;
map = map + 1;
figure;
colormap jet;imagesc(map);

dat = replace(string(map), ',', ', ');

%dlmwrite('demo1.txt',map,'delimiter',' ,')
writematrix(dat,'demo2.txt');

%fid = fopen('demo1.txt','wt');
%fprintf(fid, dat);
%fclose(fid);