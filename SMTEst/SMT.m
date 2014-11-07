classdef SMT  < handle
    %=============================================
    properties
        isLinear        = 1;        % 1 = linear solver, 2 = nonlinear solver
        tolerance       = 1E-5;     % mathemtical tolerance
        maxIterations   = 100;
    end
    properties (SetAccess = private)
        n               = 0     % number of states
        p               = 0     % number of outputs
        s               = 0     % max number of attacked sensors
        tau             = 0     % window length
        Y               = {}    % cell array of different outputs
        O               = {}    % cell array of different observability matrix
        dimNull         = []    % dimension of the null(O)
        noiseBound      = []    % noise bounds for individual sensors
        
        conflictClauses = {}    % set of conflcting learnt clauses
        agreeableClauses= {}    % set of agreeable learnt clause
        
        SATsolver       = [];   % instance of the SAT solver
        mumberOfTheoryCalls = 0;
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
        function smt = SMT()
            javapath = javaclasspath;
             if(isempty(javapath))
                SAT4J_PATH = '.\SAT4J';
                javaaddpath(fullfile(SAT4J_PATH,'org.sat4j.core.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'org.sat4j.pb.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'org.sat4j.maxsat.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'sat4j-sat.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'sat4j-pb.jar'));
                javaaddpath(fullfile(SAT4J_PATH,'sat4j-maxsat.jar'));
            end
        end
        %------------------------
        function status = init(obj, numberOfStates, numberOfSensors, windowLength, maxNumberOfAttackedSensors)
            status = 1;
            obj.SATsolver = org.sat4j.pb.SolverFactory.newLight();
            obj.n = numberOfStates; 
            obj.p = numberOfSensors; 
            obj.s = maxNumberOfAttackedSensors; 
            obj.tau = windowLength;
            
            obj.mumberOfTheoryCalls     = 0;
            numberOfBooleanVariables    = obj.p;
            numberOfConvexConstraints   = obj.p;
            
            obj.agreeableClauses        = {};
            obj.conflictClauses         = {};

            numberOfVariables = numberOfBooleanVariables + numberOfConvexConstraints;
            numberOfConstraints = numberOfConvexConstraints + 1 + length(obj.conflictClauses);

            obj.SATsolver.newVar(numberOfVariables);
            obj.SATsolver.setExpectedNumberOfClauses(numberOfConstraints);
        end
        %------------------------
        function addSensorMeasurements(obj, measurments, measurementsMatrix, noiseBound, sensorIndex)
            if(size(measurments,2) ~= 1 || size(measurments,1) ~= obj.tau)
                disp('ERROR: Check dimension of the measurments vector (Y). Measurments vector (Y) should be a (tau x 1) vector')
                return;
            elseif(size(measurementsMatrix,1) ~=obj.tau || size(measurementsMatrix,2)~= obj.n)
                disp('ERROR: Check dimension of the measurments matrix (O). Measurments matrix (O) should be a (tau x 1) vector')
                return;
            elseif(sensorIndex > obj.p)
                disp(['ERROR: Sensor Index ' num2str(sensorIndex)  ' exceeds the number of sensors.']);
                return;
            elseif(noiseBound < 0)
                disp(['ERROR: Noise bound must be positive number.']);
                return;                
            end
            
            
            obj.Y{sensorIndex}          = measurments;
            obj.O{sensorIndex}          = measurementsMatrix;
            obj.dimNull(sensorIndex)    = obj.n - rank(measurementsMatrix);
            obj.noiseBound(sensorIndex) = noiseBound;
        end
        %------------------------
        function markSensorAsSafe(obj, sensorIndex)
            obj.agreeableClauses{end+1} = sensorIndex;
        end
        %------------------------
        function  [xhat, sensorsUnderAttack ] = solve(obj)
            xhat = [];
            sensorsUnderAttack = [];
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
                    disp('ooops');
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
    %end
    %========================================
    % Helper (private) methods
    %========================================
    %methods (Access = private)
        function refreshConstraintsSAT(obj)
            obj.init(obj.n, obj.p, obj.tau, obj.s);
        end
        function writeConstriantsToSAT(obj)
            % Add the optimization goal
            % minimize b1 + b2 + ... bp
            %% TODO
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