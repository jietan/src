function channels = readRGBDN(fname,args)
    addpath('C:\Program Files\Cornell PCG\HDRITools\matlab');
    clipParam = 0.01; % To remove out of range depth values.
    
    exr = exrreadchannels(fname);
    if(strmatch('rgb',args))    
        rgb = exr.values({'color.R','color.G','color.B'});
        rgb = cat(3,rgb{:});
        channels.rgb = rgb;
    end
    
    if(strmatch('d',args))
        d = exr.values({'distance.Y'});
        d = d{1};
        d(isinf(d)) = nan;
        d(d==0) = nan;
        channels.d=d;
    end
    
    if(strmatch('n',args))
        n = exr.values({'normal.X','normal.Y','normal.Z'});
        n = cat(3,n{:});
        channels.n=n;
    end
    
    removePadding = 0;
    if(removePadding)
        % Remove padding
        fields = fieldnames(channels);
        maskArray = rmpadding(channels.(fields{1}));
        for i=2:numel(fields)
            maskArray = maskArray & rmpadding(channels.(fields{i}));
        end

        % Get bounding box of region and convert to (xmin,ymin,xmax,ymax)
        % In case banner is on in Mitsuba
        bbox = regionprops(maskArray,'BoundingBox');
        bbox = bbox.BoundingBox;
        bbox(1:2) = ceil(bbox(1:2));
        bbox(4) = bbox(2)+floor(bbox(4)-1);
        bbox(3) = bbox(1)+floor(bbox(3)-1);

        for i=1:numel(fields)
            if(ismatrix(channels.(fields{i})))
                channels.(fields{i}) = channels.(fields{i})(bbox(2):bbox(4),bbox(1):bbox(3));
            else
                channels.(fields{i}) = channels.(fields{i})(bbox(2):bbox(4),bbox(1):bbox(3),:);
            end
            channels.(fields{i})(isinf(channels.(fields{i}))) = nan;
        end
    end
%     if(isfield(channels,'n'))
%         channels.n = clipData(channels.n,[-1,1]);
%     end
    
    if(isfield(channels,'d'))
        %prctile(channels.d(:),clipParam)
        channels.d = clipData(channels.d,[prctile(channels.d(:),clipParam),prctile(channels.d(:),100-clipParam)]);
    end
end

function maskArray = rmpadding(map)
    maskArray = ~(isinf(map) | isnan(map));
    if(~ismatrix(maskArray))
        maskArray = all(maskArray,3);
    end
    
end