function outData = clipData(inData,range)
% Clips data between range(1) and range(2)
assert(range(1)<range(2));
outData = inData;
outData(outData<range(1)) = nan;
outData(outData>range(2)) = nan;
end
