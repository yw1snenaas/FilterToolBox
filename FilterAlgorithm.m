classdef(Abstract) FilterAlgorithm < handle
    % The base class of filter algorithm.
    
    properties
        KinematicModel;             % Or the transition function
        MeasurementModel;           % The measurement function
    end
    
    methods
        % Set the discrete time step of the filtering system.
        function setT(obj, t)
            if t > 0
                obj.KinematicModel.setT(t);
            end
        end
    end
    
    methods (Abstract)
        filter(this,measure);
        filter_update(this,measure);
    end
    
end

