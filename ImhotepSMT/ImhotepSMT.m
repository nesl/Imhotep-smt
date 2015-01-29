classdef ImhotepSMT  < handle
    %=============================================
    properties
        isLinear        = 1;        % 1 = linear solver, 2 = nonlinear solver
        tolerance       = 1E-5;     % mathemtical tolerance
        maxIterations   = 100;
    end
    properties (SetAccess = private)
        sys             = []    % system dynamics
        safeSensors     = []    % set of safe sensors
        n               = 0     % number of states
        p               = 0     % number of outputs
        m               = 0     % number of inputs
        s               = 0     % max number of attacked sensors
        tau             = 0     % window length
        Y_tilde         = {}    % cell array of sensor measurements
        Y               = {}    % cell array of outputs buffers
        U               = []    % input buffer
        O               = {}    % cell array of different observability matrix
        F               = {}    % cell array of input matrix
        dimNull         = []    % dimension of the null(O)
        noiseBound      = []    % noise bounds for individual sensors
        
        bufferCounter   = 0     % counter to indicate if the buffers now full
        
        conflictClauses = {}    % set of conflcting learnt clauses
        agreeableClauses= {}    % set of agreeable learnt clause
        
        SATsolver       = [];   % instance of the SAT solver
        mumberOfTheoryCalls = 0;
        
        initialized     = -1;   % check if the solver is well initialized
        isSparseObservable = -1;% does the observability check pass through
    end
    %=============================================
    properties (Hidden)
        EQUAL                   = 0;
        LESS_THAN_OR_EQUAL      = -1;
        LESS_THAN               = -2;
        GREATER_THAN_OR_EQUAL   = 1;
        GREATER_THAN            = 2;
    end
    %=============================================
    methods
        function smt = ImhotepSMT()
            javapath = javaclasspath;
             if(isempty(javapath))
                SAT4J_PATH = 'SAT4J';
                javaaddpath(fullfile(SAT4J_PATH,'org.sat4j.core.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'org.sat4j.pb.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'org.sat4j.maxsat.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'sat4j-sat.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'sat4j-pb.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'sat4j-maxsat.jar'));
            end
        end
        %------------------------
        function status = init(obj, sys, noiseBound, safeSensors, maxNumberOfAttackedSensors)
            if(obj.initialized == 1)
                disp('WARNING: Solver is already initialized. Ignoring this method call!')
                status = 1;
                return;
            end
            
            %1) Check the type of the first argument
            if(isa(sys,'ss') == 0)
                disp(['ERROR: First argument of init() is not of type "ss".' char(13) 
                'Please specify the dynamics of your system properly']);
                status = -1;
                obj.initialized = -1;
                return; 
            end
            if(isinteger(maxNumberOfAttackedSensors) == 0)
                disp('ERROR: Fourth arguments of init() must be of integer type')
                status = -1;
                obj.initialized = -1;
                return; 
            end
            
            %2) Copy the system specification and extract the dimensions
            obj.sys                     = sys;
            obj.n                       = size(sys.A,1); 
            obj.p                       = size(sys.C,1); 
            obj.m                       = size(sys.B,2);
            obj.s                       = maxNumberOfAttackedSensors; 
            obj.tau                     = obj.n;
            obj.bufferCounter           = 0;
            obj.U                       = zeros(obj.tau*obj.m,1);
            for sensorIndex = 1 : obj.p
                F = zeros(1,obj.m*obj.tau);
                for counter = 2 : obj.tau
                    F(counter,:) = [obj.sys.C(sensorIndex,:)*obj.sys.A^(counter-2)*obj.sys.B F(counter-1,1:end-obj.m)];
                end
                obj.F{sensorIndex} = F;
            end
            
            %3) Check that the second argument has the correct dimensions
            %and all its components are positive
            if(size(noiseBound,2) > size(noiseBound,1))
                noiseBound              = noiseBound';
            end
            if(sum(size(noiseBound) ~= [obj.p,1]) > 0)
                disp(['ERROR: The noise bound vector must be ' num2str(obj.p) 'x 1']);
                status                  = -1;
                obj.initialized         = -1;
                return;
            else
                for sensorIndex = 1 : obj.p
                    if(noiseBound(sensorIndex) < 0)
                        disp(['ERROR: Noise bound for sensor#' num2str(sensorIndex) ' must be positive number.']);
                        status          = -1;
                        obj.initialized = -1;
                        return;                
                    end
                    obj.noiseBound(sensorIndex) = noiseBound(sensorIndex);
                end 
            end
            
            %4) Check that the set of safe sensors are all less than obj.p
            if(max(safeSensors) > obj.p)
                disp(['ERROR: Largest index of safe sensors must be less than ' num2str(obj.p) '.']);
                status = -1;
                obj.initialized = -1;
                return;  
            end
            obj.safeSensors             = safeSensors;
            
            for counter = 1 : length(obj.safeSensors)
                obj.markSensorAsSafe(obj.safeSensors(counter));
            end
            
            %5) Calculate the observability matrix along with the
            %dimension of the null space.
            for sensorIndex = 1 : obj.p
                obj.O{sensorIndex}          = obsv(obj.sys.A, obj.sys.C(sensorIndex,:));
                obj.dimNull(sensorIndex)    = obj.n - rank(obj.O{sensorIndex});
                obj.Y{sensorIndex}          = zeros(obj.tau,1);
                obj.Y_tilde{sensorIndex}    = zeros(obj.tau,1);
            end
            
            %6) Analyze the structure of the system and check the upper
            %bound on the "maximum number of attacked sensors"
            status                          = obj.analyzeSystemStructure();
            if(status == -1)
                return;
            end
            
            %7) Initalize the SAT solver
            obj.mumberOfTheoryCalls         = 0;
            numberOfBooleanVariables        = obj.p;
            numberOfConvexConstraints       = obj.p;
            
            obj.agreeableClauses            = {};
            obj.conflictClauses             = {};
            
            numberOfVariables               = numberOfBooleanVariables + numberOfConvexConstraints;
            numberOfConstraints             = numberOfConvexConstraints + 1 + length(obj.conflictClauses);

            obj.SATsolver = org.sat4j.pb.SolverFactory.newLight();
            obj.SATsolver.newVar(numberOfVariables);
            obj.SATsolver.setExpectedNumberOfClauses(numberOfConstraints);
            
            %8) All initializations are going well
            obj.initialized                 = 1;
        end
        %------------------------
%         function addSensorMeasurements(obj, measurments, sensorIndex)
%             if(size(measurments,2) ~= 1 || size(measurments,1) ~= obj.tau)
%                 disp('ERROR: Check dimension of the measurments vector (Y). Measurments vector (Y) should be a (n x 1) vector')
%                 return;
%             elseif(sensorIndex > obj.p)
%                 disp(['ERROR: Sensor Index ' num2str(sensorIndex)  ' exceeds the number of sensors.']);
%                 return;
%             end
%             
%             
%             obj.Y{sensorIndex}          = measurments;
%             obj.dimNull(sensorIndex)    = obj.n - rank(obj.O{sensorIndex});
%         end
        %------------------------
        function [xhat, sensorsUnderAttack ]= addInputsOutputs(obj, inputs, measurments)
            xhat = zeros(obj.n,1);
            sensorsUnderAttack = [];
            
            if(sum(size(inputs) == [obj.m, 1]) < 2)
                disp(['ERROR: Check dimension of the inputs vector. Inputs vector must be a (' num2str(obj.m) 'x1) vector']);
                return;
            end
            if(size(measurments,2) ~= 1 || size(measurments,1) ~= obj.p)
                disp(['ERROR: Check dimension of the measurments vector (Y). Measurments vector (Y) must be a (' num2str(obj.p) 'x1) vector'])
                return;
            end
            
            obj.U                                 = [obj.U(obj.m+1:end); inputs];
            
            for sensorIndex = 1 : obj.p
                YY                                = obj.Y_tilde{sensorIndex};
                obj.Y_tilde{sensorIndex}          = [YY(2:end); measurments(sensorIndex)];
                obj.Y{sensorIndex}                = obj.Y_tilde{sensorIndex} - obj.F{sensorIndex}*obj.U;
            end
            
            if(obj.bufferCounter <= obj.tau)
                obj.bufferCounter                 =   obj.bufferCounter + 1;
            end
            % Is the buffers full ?
            if(obj.bufferCounter >= obj.tau)
                [xhat, sensorsUnderAttack ]         = obj.solve();
                % run the estimate forward in time
                for counter = 1 : obj.tau
                    xhat = obj.sys.A*xhat + obj.sys.B*obj.U((counter-1)*obj.m + 1: (counter-1)*obj.m  +obj.m);
                end
            end
        end
        
        %------------------------
        function status = checkObservabilityCondition(obj, tol)
            if(obj.initialized == -1)
                disp('ERROR: The solver is not initalized yet. To initialize the solver, call the init() method');
                status = -1;
                return;
            end
            
            if(obj.s == 0)
                disp('The specified number of sensors under attack is 0. Abort');
                status = 0;
                return;
            end
            
            OO_lower = [];
            disp('Calculating lower bound on the number of sensors that can be attacked...');
            for counter = 1 : obj.p
                [~, idx]                = obj.licols(obj.O{counter}, tol);
                OO_temp                 = zeros(1,obj.n); 
                OO_temp(idx)            = 1;
                OO_lower                = [OO_lower; OO_temp];
            end
            s_lower_bound               = floor((min(sum(OO_lower,1)) - 1)/2);
            disp(['lower bound is euqal to ' num2str(s_lower_bound)]);
            
            disp(' ');
            if(obj.s <= s_lower_bound)
                disp('The specified number of sensors under attack is below the lower bound');
                obj.isSparseObservable  = 1;
                status                  = 1;
                return
            end
            
            max_count                   = nchoosek(obj.p, obj.p - length(obj.safeSensors) - 2*obj.s);
            allCombinations             = combnk(setdiff(1:obj.p, obj.safeSensors), 2*obj.s);
            disp(' ');
            disp('The specified "number of sensors under attack" lies between the theortical');
            disp('upper and lower bounds ...');
            disp(['Checking the observability of ' num2str(max_count) ' combinations ... This may take some time']);
            observable = 1;
            disp(' ');
            for counter = 1 : max_count
                sensorsIdx = setdiff(1:obj.p, allCombinations(counter,:));
                if(rank( obsv(obj.sys.A, obj.sys.C(sensorsIdx,:))) < obj.n )
                    observable = 0;
                    disp(['Iteration number ' num2str(counter) ' out of ' num2str(max_count) ' combinations ... FAIL!']);
                    break;
                end
                disp(['Iteration number ' num2str(counter) ' out of ' num2str(max_count) ' combinations ... PASS!']);
            end
            if(observable == 0)
                disp('Sparse observability condition failed. Re-initialize the solver by calling init().');
                status                  = -1;
                obj.isSparseObservable  = -1;
                obj.initialized         = -1;
                return;
            else
                disp('Sparse observability condition passed! Solver is ready to run!');
                status                  = 1;
                obj.isSparseObservable  = 1;
                obj.initialized         = 1;
            end
        end
    end
    %========================================
    % Helper (private) methods
    %========================================
    methods (Access = private)
        function status = analyzeSystemStructure(obj)
            status          = -1;
            if(obj.s < 0)
                disp('ERROR: the specified "number of sensors under attack" must be positive');
                return;
            end
            
            OO_upper        = [];
            
            for sensorIndex = 1 : obj.p
                OO_temp     = sum(obj.O{sensorIndex}, 1);
                OO_upper    = [ OO_upper; (OO_temp > 0) + (OO_temp < 0)];
            end
            
            % 1-Theoritical upper bound
            idx             = [];
            for counter = 1 : length(obj.safeSensors)
                idx = [idx find(OO_upper(obj.safeSensors(counter),:) == 1)];
            end
            unsafeStates        = setdiff([1:obj.n], idx);
            stateSecurityIndex  = sum(OO_upper(:,unsafeStates), 1);
            
            if(isempty(stateSecurityIndex))
                s_upper_bound   = obj.p - length(obj.safeSensors);
            else
                s_upper_bound   = floor((min(stateSecurityIndex) - 1)/2);
            end
            
            
            if(s_upper_bound < 0 )
                s_upper_bound = 0;
            end
            reportStr1      = ['Theoretical upper bound on maximum number of attacks is: ' num2str(s_upper_bound)];
            disp(reportStr1);
            
            % 2-structure of sensors
            index           = find(stateSecurityIndex == min(stateSecurityIndex), 1, 'first');
            sensorIndex     = find(OO_upper(:,index) == 1);
            reportStr2      = 'Sensors that can improve system security are: ';
            
            if(isempty(sensorIndex) == 0)
                for counter = 1 : length(sensorIndex)
                    reportStr2 = [reportStr2 'Sensor#' num2str(sensorIndex(counter)) ' '];
                end
                disp(reportStr2);
            end
            
            disp(' ');
            if(s_upper_bound < obj.s)
                disp('ERROR: System structure does not match the specified maximum number of attacked sensors');
                disp(['Maximum number of attacked sensors must be less than ' num2str(s_upper_bound)]);
                status      = -1;
                return;
            elseif(obj.s > 0)
                disp('Disclaimer: Correction of the solver outputs is guaranteed if and only if');
                disp(['the specified system is observable after removing all combinations of ' num2str(2*obj.s) ' unsafe sensors.']);
                disp('To run this combinatorial test, call the checkObservabilityCondition()');
            else
                disp('This system can not tolerate any sensor attacks.');
                status      = 0;
                return;
            end
            status          = 1;
        end
        %------------------------
        function [Xsub, idx] = licols(obj, X,tol)
            %Extract a linearly independent set of columns of a given matrix X
            % [Xsub,idx]=licols(X)
            %
            %in:
            %
            % X: The given input matrix
            % tol: A rank estimation tolerance. Default=1e-10
            %
            %out:
            %
            % Xsub: The extracted columns of X
            % idx: The indices (into X) of the extracted columns

           if ~nnz(X) %X has no non-zeros and hence no independent columns
               Xsub=[]; idx=[];
               return
           end
            if nargin<2, tol=1e-25; end           
            [~, R, E] = qr(X,0);
            if ~isvector(R)
                diagr = abs(diag(R));
            else
                diagr = R(1); 
            end
            %Rank estimation
            r = find(diagr >= tol*diagr(1), 1, 'last'); %rank estimation
%            r = rank(X);

            idx=sort(E(1:r));
            Xsub=X(:,idx);
        end
        %------------------------
        function markSensorAsSafe(obj, sensorIndex)
            obj.agreeableClauses{end+1} = sensorIndex;
        end
        %------------------------
        function  [xhat, sensorsUnderAttack ] = solve(obj)
            xhat = [];
            sensorsUnderAttack = [];
            
            if(obj.initialized == -1)
                disp('ERROR: System properties are not well defined. TERMINATE');
                return;
            end
            
            solved = 0;
            iterationCounter = 1;
            while solved == 0 || iterationCounter >= obj.maxIterations
                obj.writeConstriantsToSAT();
                iterationCounter    = iterationCounter + 1;
                
                % Get the boolean assignment
                status = obj.SATsolver.isSatisfiable();
                if(status == 0)
                    disp('ERROR: System is UNSAT. Check the system parameters and measurements.');
                    return
                end
                
                % Check which convex constraints needs to be satisifed
%                 convexConstraintAssignment = zeros(1, obj.p);
%                 for counter = 1 : obj.p
%                     convexConstraintAssignment(counter) = calllib('CalCSminisatp','getModel', ['c' num2str(counter)]);
%                 end
                
                convexConstraintAssignment  = obj.SATsolver.model;
                constraints         = find(convexConstraintAssignment < 0);
                sensorsUnderAttack  = find(convexConstraintAssignment > 0);
                
                Y_active = []; O_active = [];
                for counter = 1 : length(constraints)
                    Y_active = [Y_active; obj.Y{constraints(counter)}];
                    O_active = [O_active; obj.O{constraints(counter)}];
                end
                % Formalize the Convex optimization problem
                %xhat = linsolve(O_active,Y_active);
                xhat = O_active\Y_active;
                
                % Calculate the individual slack vriables
                slack = zeros(1,obj.p);
                for counter = 1 : length(constraints)
                    slack(constraints(counter)) = norm(obj.Y{constraints(counter)} - obj.O{constraints(counter)}*xhat);
                end

                % Check if the convex constraints are SAT
                if sum(slack) <= obj.tolerance + sum(obj.noiseBound(constraints));
                    solved = 1;
                    return;
                end

                % If UNSAT, then add the conflict clause to the Problem
                foundConflict = 0;  conflicts = [];
                foundAgreeable = 0; agreeable = [];
                
                
                % 1- Sort based on slack
                [~, slackIndex] = sort(slack(constraints), 'ascend');
                slackIndex      = constraints(slackIndex);
                
                % 2- We know that at most s-sensors are under attack.
                % The worst case scenario is that all of them are 
                % present in the current assignment. So skip the highest
                % s-slac and for the rest, sort them according to their
                % diemnsion of null space. Remeber that the boolean
                % assignment already excluded s sensors. Now by excluding
                % more s sensors, we have (p - 2s) remaining sensors. From
                % the 2s-sparse observability condition we know that any
                % (2s - p) sensors are fully observable, i.e. their
                % observability matrix spans the full space.
                
                indexLowSlackSensors            = slackIndex(1 : end - obj.s);
                [~, indexSortedLowSlackSensors] = sort(obj.dimNull(indexLowSlackSensors), 'ascend');
                indexSortedLowSlackSensors      = indexLowSlackSensors(indexSortedLowSlackSensors);
                % Use tha (2s - p) sensors against the sensors with the max
                % slack to generate a short conflicting clause
                max_slack_index = slackIndex(end);
                
                
                % search linearly for a sensor that
                % conflicts with the max slack.
                Y_active = obj.Y{max_slack_index};
                O_active = obj.O{max_slack_index};
                for counter = 1 : length(indexSortedLowSlackSensors)
                    sensor      = indexSortedLowSlackSensors(counter);
                    Y_active    = [Y_active;  obj.Y{sensor}];
                    O_active    = [O_active;  obj.O{sensor}];
                    %xhat        = linsolve(O_active,Y_active);
                    xhat        = O_active\Y_active;
                    if(norm(Y_active - O_active * xhat) > obj.tolerance + sum(obj.noiseBound(indexSortedLowSlackSensors(1:counter))) )
                        % Conflict discovered
                        conflicts = [max_slack_index, indexSortedLowSlackSensors(1:counter)'];
                        foundConflict = 1;
                        break;
                    end
                end

%                 if(foundConflict == 0)
%                     for counter = setdiff(constraints, max_slack_index)
%                         Y_active    = [obj.Y{max_slack_index}; obj.Y{counter}];
%                         O_active    = [obj.O{max_slack_index}; obj.O{counter}];
%                         %xhat        = linsolve(O_active,Y_active);
%                         xhat        = O_active\Y_active;
%                         if(norm(Y_active - O_active * xhat) > obj.tolerance)
%                             % Conflict discovered
%                             conflicts = [max_slack_index, counter];
%                             foundConflict = 1;
%                             break;
%                         end
%                     end
%                 end
                

                % Just-in-case if the previous search failed, then use the
                % weakest clause
                if(foundConflict == 0)
                    %disp('ooops');
                    conflicts       = constraints;
                end
                obj.conflictClauses{end+1} = conflicts;
                
                % Search for agreeable constraints. The longest the better
                % start by lowest slack and go linearly until you find the
                % longest set of agreeable constraints
                Y_active = []; O_active = [];
                for counter = 1 : length(constraints)
                    sensor = slackIndex(counter);
                    Y_active    = [Y_active;  obj.Y{sensor}];
                    O_active    = [O_active;  obj.O{sensor}];
                    %xhat        = linsolve(O_active,Y_active);
                    xhat        = O_active\Y_active;
                    if(norm(Y_active - O_active * xhat) > obj.tolerance + sum(obj.noiseBound(slackIndex(1:counter))) )
                        % Conflict discovered
                        if(counter > obj.p - 2*obj.s)
                            agreeable = slackIndex(1:counter-1);
                            foundAgreeable = 1;
                        end
                        break;    
                    end
                end
                if(foundAgreeable)
                    obj.agreeableClauses{end+1} = agreeable;
                end
                %obj.init(obj.n, obj.p, obj.tau, obj.s);
            end
            
            obj.mumberOfTheoryCalls = iterationCounter;
            if solved == 0
                xhat = [];
                sensorsUnderAttack = [];
                disp('ERROR: Maximum number of iterations reached.');
                return;
            end
        end
        %------------------------
        function writeConstriantsToSAT(obj)
            % Add the optimization goal
            % minimize b1 + b2 + ... bp
            % TODO
%             calllib('CalCSminisatp','startGoal')
%             for counter = 1 : obj.p
%                 calllib('CalCSminisatp','addToConstraint', ['b' num2str(counter)], 1);
%             end
%             calllib('CalCSminisatp','closeGoal');


            
            % Add the constraint on the number of attacks does not exceed s
            % i.e., b1 + b2 + ... + bp  <= s
            literals = []; coeffs = [];
            %calllib('CalCSminisatp','startNewConstraint')
            for counter = 1 : obj.p
                literals(end+1) = counter;
                coeffs(end+1) = 1;
            end
            litI = org.sat4j.core.VecInt(literals); 
            coefI = org.sat4j.core.VecInt(coeffs);
            obj.SATsolver.addAtMost(litI, coefI, obj.s);

            
%             % For each convex constriant, define a boolean variable 
%             % (e.g. c1 = ||Y_1 - O_1 x|| \le 0). 
%             % Define the following boolean constraint: (not bi => ci)
%             % This constraint reduces to: (bi or ci) which in turn can be written
%             % as: bi + ci >= 1
%             for counter = 1 : obj.p
%                 calllib('CalCSminisatp','startNewConstraint')
%                 calllib('CalCSminisatp','addToConstraint', ['b' num2str(counter)], 1);
%                 calllib('CalCSminisatp','addToConstraint', ['c' num2str(counter)], 1);
%                 calllib('CalCSminisatp','closeConstraint', obj.GREATER_THAN_OR_EQUAL, 1);
%             end


            % Add the conflicting clauses
            %
            % conflict cluases take the form
            % b_i + b_j >= 1
            % which inforces that both of these sensors can not be
            % un-attacked at the same time. Either both are attacked (so
            % the SAT solver shall assign both to 1), or one of them under
            % attack while the other is not, the SAT solver is going to
            % assign 0 for one of them and 1 to the other.
            
            for counter = 1 : size(obj.conflictClauses,2)
                literals = []; coeffs = [];
                for counter2 = 1 : length(obj.conflictClauses{counter})
                    literals(end+1) = obj.conflictClauses{counter}(counter2);
                    coeffs(end+1) = 1;
                end
                litI = org.sat4j.core.VecInt(literals); 
                coefI = org.sat4j.core.VecInt(coeffs);
                obj.SATsolver.addAtLeast(litI, coefI, 1);
            end

            % Add the agreeable clauses
            %
            % agreeable cluases take the form
            % b_i + b_j == 0
            % which inforces that this set of sensors to be all in the
            % set of un-attacked sensors.
            
            for counter = 1 : size(obj.agreeableClauses,2)
                literals = []; coeffs = [];
                for counter2 = 1 : length(obj.agreeableClauses{counter})
                    literals(end+1) = obj.agreeableClauses{counter}(counter2);
                    coeffs(end+1) = 1;
                end
                litI = org.sat4j.core.VecInt(literals); 
                coefI = org.sat4j.core.VecInt(coeffs);
                obj.SATsolver.addExactly(litI, coefI, 0);
            end
        end
    end
end