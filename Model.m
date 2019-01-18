classdef (Abstract) Model < handle
    % The base class of maneuvering model and measurement model.
    
    properties (SetAccess = protected)
        Std;
        T;
        CovMatrix;
    end
    
    methods (Abstract)
        setT(this, t);
        setStd(this, std);
        setCovMatrix(this, matrix);
        estimateNewState(this, x_old);
        addDisturb(this, x);
        predictNewState(this, x_old);
        getLinear(this, x, t);
        getCovMatrix(this, t);
    end
    
end

