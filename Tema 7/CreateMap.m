function [Map,colors] = CreateMap(MapSize,nFeatures)
Map = zeros(2,nFeatures);
colors = zeros(nFeatures,3);

for i_feat = 1:nFeatures
    Map(:,i_feat) = MapSize*rand(2,1)-MapSize/2;
    colors(i_feat,:) = [rand rand rand];
end
end
