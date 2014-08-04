folderName = 'C:\Users\JieTan\Documents\MyProjects\AdobeIntern\PrincetonModelNet\chair\chair_000000280_collada\depth\';
files=dir(folderName);
for k = 1 : length(files)
   fileName = files(k).name;
   if (endswith(fileName, '.exr'))
       channel = readRGBDN(strcat(folderName,fileName), 'd');
       outImage = kinectPollute(channel.d);
       outImage = uint16(outImage * 10);
       imshow(outImage, []);
       figure;
       imwrite(outImage, strcat(folderName, fileName, '.png'));
   end
end