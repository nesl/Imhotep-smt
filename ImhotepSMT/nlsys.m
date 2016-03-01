%  * Copyright (c) 2015 The Regents of the University of California.
%  * All rights reserved.
%  *
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions
%  * are met:
%  * 1. Redistributions of source code must retain the above copyright
%  *    notice, this list of conditions and the following disclaimer.
%  * 2. Redistributions in binary form must reproduce the above
%  *    copyright notice, this list of conditions and the following
%  *    disclaimer in the documentation and/or other materials provided
%  *    with the distribution.
%  * 3. All advertising materials mentioning features or use of this
%  *    software must display the following acknowledgement:
%  *       This product includes software developed by Networked &
%  *       Embedded Systems Lab at UCLA
%  * 4. Neither the name of the University nor that of the Laboratory
%  *    may be used to endorse or promote products derived from this
%  *    software without specific prior written permission.
%  *
%  * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS''
%  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
%  * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
%  * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS
%  * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
%  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
%  * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
%  * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
%  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
%  * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
%  * SUCH DAMAGE.
%  */

%   This filw is a definition for the data structure "NonLinear System
%   (nlsys)".

classdef nlsys  < handle
    properties (SetAccess = private)
        n           = 0;                    % number of states
        p           = 0;                    % number of outputs
        m           = 0;                    % number of inputs
        tau         = 0;                    % window length required by observer to operate
        f           = @(x,u)[];             % handle to dynamics map f: state x input -> state
        h           = @(x)[];               % handle to output ma h: state -> output
        observer    = @(sensorIndex,y,u)[]; % handle to obsever Obsv: outputs x input -> state
        isObservable= @(sensorIndex)[];     % handle to check obser: sensorIndex -> Yes/No
    end
    %=============================================
    methods
        function sys = nlsys()
        end
        %------------------------
        function init(obj, f, h, observer, isObservable,n,p,m,tau)
            if(isa(f,'function_handle') == 0)
                disp('ERROR: First argument is not a function handle');
            end
            
            if(isa(h,'function_handle') == 0)
                disp('ERROR: Second argument is not a function handle');
            end
            
            if(isa(observer,'function_handle') == 0)
                disp('ERROR: Third argument is not a function handle');
            end
            
%             if(isa(isObservable,'function_handle') == 0 && isempty(isObservable))
%                 disp('ERROR: Fourth argument is not a function handle or not empty.');
%             end
            obj.n           = n;
            obj.p           = p;
            obj.m           = m;
            obj.tau         = tau;
            obj.f           = f;
            obj.h           = h;
            obj.observer    = observer;
            obj.isObservable=isObservable;
        end
    end
end