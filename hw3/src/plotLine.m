
%% Function plotLine
%  Plots a line from given two points and color
function plotLine(v1,v2,c)
    v=[v2;v1];
    plot3(v(:,1),v(:,2),v(:,3),c);
end