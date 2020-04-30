function [ treePath, treePathCDF ] = getTreePath(Q)

pathQ = NaN(Q.numJoints, Q.nodePos);
pathCDF = NaN(1, Q.nodePos);

k = Q.nodePos;
currNode = Q.nodes(:,k);
currParent = Q.parents(k);
pathCDF(k) = Q.cdf(k);

while ~isnan(currParent)
    pathQ(:,k) = currNode;
    pathCDF(k-1) = Q.cdf(currParent);
    currNode = Q.nodes(:,currParent);
    currParent = Q.parents(currParent);
    k = k - 1;
end

treePath = pathQ(:,all(~isnan(pathQ)));
treePathCDF = pathCDF(~isnan(pathCDF));

end

