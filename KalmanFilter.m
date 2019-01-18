classdef KalmanFilter < FilterAlgorithm
    % The implementation of Kalman filter.
    % It could be used to estimate the target's state of discrete time.
    
    properties (SetAccess = protected)
        P;              % The covariance of target's state.
        X_init;         % The initial value of target's state.
        X_est;          % The estimation of target's state at every time.
    end
    
    methods
        %% Constructor
        % Input value:
        % kinematicmodel: The transition function.
        % measurementmodel: The measurement function.
        % xinit: The initial value of target's state.
        % pinit: The covariance of target's state at start.
        function td = KalmanFilter(kinematicmodel, measurementmodel, xinit, pinit)
            if nargin == 0
                td.KinematicModel = CAmodel(1, [10, 10, 10]);
                td.MeasurementModel = Cartesian(1, [10, 10, 10], 'CAmodel');
                td.P = zeros(9,9);
                td.X_init = zeros(9,1);
                td.X_est = td.X_init;
            elseif nargin == 2
                td.KinematicModel = kinematicmodel;
                td.MeasurementModel = measurementmodel;
                td.P = kinematicmodel.targetInit_P();
                td.X_init = kinematicmodel.targetInit_X();
                td.X_est = td.X_init;
            elseif nargin == 3
                td.KinematicModel = kinematicmodel;
                td.MeasurementModel = measurementmodel;
                dim = length(xinit);
                td.P = zeros(dim, dim);
                td.X_init = xinit;
                td.X_est = xinit;
                try
                    temp = kinematicmodel.estimateNewState(xinit);
                catch
                    disp('The initial state and the model are not compared.');
                    td = 0;
                end
            elseif nargin == 4
                td.KinematicModel = kinematicmodel;
                td.MeasurementModel = measurementmodel;
                td.P = pinit;
                td.X_init = xinit;
                td.X_est = xinit;
                try
                    temp = kinematicmodel.estimateNewState(xinit);
                catch
                    disp('The initial state and the model are not compared.');
                    td = 0;
                end
            else
                disp('The number of input should be right.');
                td = 0;
            end
        end
        %% Set function
        % Set the initial state of target, it could change both X_init and
        % X_est.
        function setXinit(this,xinit)
            if size(this.X_init, 1) == size(xinit, 1) && size(this.X_init,2) == size(xinit,2)
                this.X_init = xinit;
                this.X_est = xest;
            end
        end
        % Set the target's state at present.
        function setXest(this,x)
            if size(this.X_est, 1) == size(x,1) && size(this.X_est, 2) == size(x,2)
                this.X_est = x;
            end
        end
        % Set the covariance of target's state.
        function setP(this, matrix)
            if size(this.P, 1) == size(matrix,1) && size(this.P,2) == size(matrix,2)
                % Make sure that new matrix is a positive semidefinite
                % matrix.
                eig_matrix = eig(matrix);
                if isempty(find(eig_matrix < 0))
                    this.P = matrix;
                end
            end
        end
        %% One step filter.
        % The measurement is a colume vector.
        function [x_est_new, e, S] = filter_update(this,measure)
            F = this.KinematicModel.getLinear(this.X_est);
            Q = this.KinematicModel.CovMatrix;
            R = this.MeasurementModel.CovMatrix;
            % Time upate
            x_pre = this.KinematicModel.estimateNewState(this.X_est);
            P_pre = F * this.P * F' + Q;
            % Measurement update
            H = this.MeasurementModel.getLinear(x_pre);
            S = H * P_pre * H' + R;
            K = P_pre * H' * pinv(S);
            e = measure - this.MeasurementModel.estimateNewState(x_pre);
            x_est_new = x_pre + K * e;
            this.X_est = x_est_new;
            dim = size(P_pre,1);
            this.P = (eye(dim) - K * H) * P_pre;
        end
        %% Obtain the filtering result of the whole simulation period
        % Each colume of measurement denotes the measurement at
        % corresponding state.
        function x_est = filter(this,measure)
            timeLength = size(measure, 2);
            dim = size(this.KinematicModel.getLinear(this.X_est,1));
            x_est = zeros(dim, timeLength);
            this.X_est = this.X_init;
            for k = 1:timeLength
                [x_est(:,k),~,~] = this.filter_update(measure(:,k));
            end
        end
    end
    
end

