function pollutedImage = kinectPollute(depthImage)
pd = makedist('Normal');
height = size(depthImage, 1);
width = size(depthImage, 2);

xi = 1:height;
xj = 1:width;

udIdx = repmat(xj, height,1);
lfIdx = repmat(xi',1, width);

 
lrShiftImage = 0.5 * random(pd, height, width);
udShiftImage = 0.5 * random(pd, height, width);
shuffledDepth = interp2(depthImage, udIdx + udShiftImage, lfIdx + lrShiftImage);
disparityNoise = random(pd, height, width);
magicConstant = 35130.0; %http://www.cs.berkeley.edu/~barron/BarronMalikCVPR2013_supp.pdf

disparityImage = floor(magicConstant ./ shuffledDepth + 1.0 / 6.0 * disparityNoise + 0.5);
pollutedImage = magicConstant ./ disparityImage;
% for i = 1 : height
%     fprintf('%d\n', i);
%     for j = 1 : width
%         if depthImage(i, j) < 0.01
%             pollutedImage(i, j) = 0;
%         else
% %             lateralShiftI = 0.5 * random(pd);
% %             lateralShiftJ = 0.5 * random(pd);
% %             disparity = magicConstant / interpImg(depthImage, [i + lateralShiftI, j + lateralShiftJ]);
%             disparity = magicConstant / depthImage(i, j);
%             disparity = floor(disparity + 1.0 / 6.0 * random(pd) + 0.5);
%             pollutedImage(i, j) = magicConstant / disparity;
%         end
%     end
% end
