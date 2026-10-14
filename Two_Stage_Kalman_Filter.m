classdef Two_Stage_Kalman_Filter < handle
    % TWO_STAGE_KALMAN_FILTER - Fault detection and diagnosis (FDD) unit
    
    %
    % Estimates the actuator loss of effectiveness factor gamma in [0, 1]:
    %   gamma = 0         : Fault-free (normal operation)
    %   0 < gamma < 0.65  : Non-severe fault (degraded performance / motion re-coordination)
    %   0.65 <= gamma <= 1: Severe fault (robot leaves formation / task reassignment)

    properties
        % Discrete-time system matrices 
        A;          % 4x4 state transition matrix (A_k)
        B;          % 4x2 control input matrix (B_k)
        C;          % 2x4 output measurement matrix (C_k)

        % Noise covariance matrices
        Qx;         % 4x4 state process noise covariance (Q_k^x)
        Qgamma;     % 2x2 fault parameter process noise covariance (Q_k^gamma)
        R;          % 2x2 measurement noise covariance (R_k)

        % Filter internal states & covariances
        x_tilde;    % 4x1 fault-free state estimate (\tilde{x}_{k|k})
        Px_tilde;   % 4x4 fault-free error covariance (\tilde{P}_{k|k}^x)

        gamma_hat;  % 2x1 estimated fault parameter vector [\gamma_R; \gamma_L] (\hat{\gamma}_{k|k})
        Pgamma;     % 2x2 fault parameter error covariance (P_{k|k}^\gamma)

        V;          % 4x2 two-stage coupling matrix (V_{k|k})

        % Compensated full state & covariance 
        x_hat;      % 4x1 compensated state estimate [\omega_R; i_R; \omega_L; i_L] (\hat{x}_{k|k})
        P_hat;      % 4x4 compensated error covariance (P_{k|k})

        % Physical DC motor parameters 
        Kc = 0.2;   % Motor torque and back-EMF constant (N.m/A or V.s/rad)
        Im = 0.01;  % Rotor moment of inertia (kg.m^2)
        b  = 0.1;   % Motor viscous friction constant (N.m.s/rad)
        La = 0.05;  % Armature inductance (H)
        Ra = 2.0;   % Armature resistance (Ohm)
        Ts = 0.1;   % Sampling time (s)
    end

    methods
        function obj = Two_Stage_Kalman_Filter(paramsOrTs)
            % Constructor: Accepts either a 'params' struct or a scalar 'Ts'
            if nargin >= 1 && ~isempty(paramsOrTs)
                if isnumeric(paramsOrTs) && isscalar(paramsOrTs)
                    obj.Ts = paramsOrTs;
                elseif isstruct(paramsOrTs) || isobject(paramsOrTs)
                    if isfield(paramsOrTs, 'Ts'), obj.Ts = paramsOrTs.Ts;
                    elseif isfield(paramsOrTs, 'dt'), obj.Ts = paramsOrTs.dt;
                    elseif isfield(paramsOrTs, 'sampleTime'), obj.Ts = paramsOrTs.sampleTime;
                    end
                    if isfield(paramsOrTs, 'Kc'), obj.Kc = paramsOrTs.Kc; end
                    if isfield(paramsOrTs, 'Im'), obj.Im = paramsOrTs.Im; end
                    if isfield(paramsOrTs, 'b'),  obj.b  = paramsOrTs.b;  end
                    if isfield(paramsOrTs, 'La'), obj.La = paramsOrTs.La; end
                    if isfield(paramsOrTs, 'Ra'), obj.Ra = paramsOrTs.Ra; end
                end
            end

            % Continuous-time state-space model for TWO driving motors 
            % State vector: x = [omega_R; i_R; omega_L; i_L] (n = 4)
            % Input vector: u = [E_R; E_L]                   (m = 2)
            % Output vector: y = [omega_R; omega_L]          (p = 2)
            A_cont = [-obj.b/obj.Im,   obj.Kc/obj.Im,  0,              0;
                -obj.Kc/obj.La, -obj.Ra/obj.La,  0,              0;
                0,              0,             -obj.b/obj.Im,   obj.Kc/obj.Im;
                0,              0,             -obj.Kc/obj.La, -obj.Ra/obj.La];

            B_cont = [0,          0;
                1/obj.La,   0;
                0,          0;
                0,          1/obj.La];

            C_cont = [1, 0, 0, 0;
                0, 0, 1, 0];

            % Discretize with Zero-Order Hold 
            sys = ss(A_cont, B_cont, C_cont, zeros(2, 2));
            sysd = c2d(sys, obj.Ts);

            obj.A = sysd.A;
            obj.B = sysd.B;
            obj.C = sysd.C;

            % Initialize noise covariances (tuneable based on sensor/process noise)
            obj.Qx     = 1e-4 * eye(4);
            obj.Qgamma = 1e-3 * eye(2); % Allows tracking time-varying step faults in gamma
            obj.R      = 1e-2 * eye(2);

            % Initialize estimates and covariances at k = 0
            obj.x_tilde   = zeros(4, 1);
            obj.Px_tilde  = eye(4);

            obj.gamma_hat = zeros(2, 1); % Initially fault-free: [gamma_R; gamma_L] = [0; 0]
            obj.Pgamma    = 0.1 * eye(2);

            obj.V         = zeros(4, 2);

            obj.x_hat     = zeros(4, 1);
            obj.P_hat     = eye(4);
        end

        function [gamma, x_comp] = estimate(obj, u, y, ~)
            % ESTIMATE - Performs one prediction-correction cycle of the TSKF
            %
            % Inputs:
            %   u - 2x1 (or scalar) Control input voltage [E_R; E_L] at step k
            %   y - 2x1 (or scalar) Measured wheel speed [omega_Ra; omega_La] at step k+1
            %
            % Outputs:
            %   gamma  - 2x1 Estimated loss of effectiveness [gamma_R; gamma_L] in [0, 1]
            %   x_comp - 4x1 Compensated state estimate [omega_R; i_R; omega_L; i_L]

            % Support both 2x1 [Right; Left] vectors and scalar fallback
            u = u(:);
            y = y(:);
            if isscalar(u), u = [u; u]; end
            if isscalar(y), y = [y; y]; end

            % Construct diagonal control input matrix U_k 
            U_k = diag(u(1:2)); % [u_k^1, 0; 0, u_k^2]

            %% 1. Fault Parameter Time-Update / Prediction 
            gamma_pred  = obj.gamma_hat;            % \hat{\gamma}_{k+1|k}
            Pgamma_pred = obj.Pgamma + obj.Qgamma;  % P_{k+1|k}^\gamma

            %% 2. Prediction Coupling Equations 
            % E_k = -B_k * U_k
            W_k    = obj.A * obj.V - obj.B * U_k;              
            V_pred = W_k * obj.Pgamma / Pgamma_pred;           

            %% 3. Fault-Free State Time-Update / Prediction 
            x_tilde_pred = obj.A * obj.x_tilde + obj.B * u(1:2) ...
                + W_k * obj.gamma_hat - V_pred * obj.gamma_hat; % 

            Px_tilde_pred = obj.A * obj.Px_tilde * obj.A' + obj.Qx ...
                + W_k * obj.Pgamma * W_k' ...
                - V_pred * Pgamma_pred * V_pred';              

            %% 4. Filter Residual and Covariance 
            S_tilde = obj.C * Px_tilde_pred * obj.C' + obj.R;   %\tilde{S}_{k+1}
            r_tilde = y(1:2) - obj.C * x_tilde_pred;            %\tilde{r}_{k+1}

            %% 5. Fault-Free State Measurement-Update 
            Kx_tilde     = (Px_tilde_pred * obj.C') / S_tilde;             
            obj.x_tilde  = x_tilde_pred + Kx_tilde * r_tilde;              
            obj.Px_tilde = (eye(4) - Kx_tilde * obj.C) * Px_tilde_pred;    

            %% 6. Update Coupling Equations 
            H_pred = obj.C * V_pred;                            
            obj.V  = V_pred - Kx_tilde * H_pred;                

            %% 7. Fault Parameter Measurement-Update 
            S_gamma = H_pred * Pgamma_pred * H_pred' + S_tilde;
            K_gamma = (Pgamma_pred * H_pred') / S_gamma;                   

            obj.gamma_hat = gamma_pred + K_gamma * (r_tilde - H_pred * obj.gamma_hat); 
            obj.Pgamma    = (eye(2) - K_gamma * H_pred) * Pgamma_pred;      

            %% 8. Compensated State and Covariance Estimator 
            obj.x_hat = obj.x_tilde + obj.V * obj.gamma_hat;     
            obj.P_hat = obj.Px_tilde + obj.V * obj.Pgamma * obj.V';         

            % Clamp output gamma to valid physical bounds [0, 1] 
            gamma  = max(min(obj.gamma_hat, 1), 0);
            x_comp = obj.x_hat;
        end
    end


end