%qpOASES -- An Implementation of the Online Active Set Strategy.
%Copyright (C) 2007-2011 by Hans Joachim Ferreau, Andreas Potschka,
%Christian Kirches et al. All rights reserved.
%
%qpOASES is distributed under the terms of the
%GNU Lesser General Public License 2.1 in the hope that it will be
%useful, but WITHOUT ANY WARRANTY; without even the implied warranty
%of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%See the GNU Lesser General Public License for more details.
%
%---------------------------------------------------------------------------------
%
%qpOASES_sequenceSB is intended to solve a sequence of simply bounded
%convex quadratic programming (QP) problems of the following form:
%
%                min   0.5*x'Hx + x'g
%                s.t.  lb  <=  x <= ub
%
%I) Call
%     [x,fval,exitflag,iter,lambda] = ...
%                      qpOASES_sequenceSB( 'i',H,g,lb,ub,{x0,{options}} )
%for initialising and solving the first above-mentioned QP of the sequence
%starting from an initial guess x0. H must be a symmetric and positive definite 
%matrix and all vectors g, lb, ub, have to be given as column vectors. 
%If no initial guess is provided, iterations start from the origin. Options can
%be generated using the qpOASES_options command, otherwise default values are used.
%
%II) Call
%     [x,fval,exitflag,iter,lambda] = ...
%                      qpOASES_sequenceSB( 'h',g,lb,ub,{options} )
%for hotstarting from the previous QP solution to the one of the next QP
%given by the vectors g, lb, ub. Options can be generated using the 
%qpOASES_options command, otherwise default values are used.
%
%III) Having solved the last QP of your sequence, call
%     qpOASES_sequenceSB( 'c' )
%in order to cleanup the internal memory.
%
%
%Optional Outputs (only obj is mandatory):
%    x         -  optimal primal solution vector   (if status==0)
%    fval      -  optimal objective function value (if status==0)
%    exitflag  -   0: QP solved
%                  1: QP could not be solved with given number of iterations
%                 -1: QP could not be solved due to an internal error
%                 -2: QP is infeasibile and thus could not be solved 
%                 -3: QP is unbounded and thus could not be solved 
%    iter      -  number of active set iterations actually performed
%    lambda    -  optimal dual solution vector     (if status==0)
%
%
%See also QPOASES_OPTIONS, QPOASES_SEQUENCE, QPOASES_SEQUENCEVM
%
%
%For additional information see the qpOASES User's Manual or
%visit http://www.qpOASES.org/.
%
%Please send remarks and questions to support@qpOASES.org!