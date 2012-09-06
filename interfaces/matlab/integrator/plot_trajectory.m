%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%    Developed within the Optimization in Engineering Center (OPTEC) under
%    supervision of Moritz Diehl. All rights reserved.
%
%    ACADO Toolkit is free software; you can redistribute it and/or
%    modify it under the terms of the GNU Lesser General Public
%    License as published by the Free Software Foundation; either
%    version 3 of the License, or (at your option) any later version.
%
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%    Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ACADO Toolkit; if not, write to the Free Software
%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
%    Author: Niels Haverbeke, Boris Houska, Hans Joachim Ferreau, David Ariens
%    Date: 2008-2010


function [ ] = plot_trajectory( trajectory,statesIdx,useSubplots,plotType,isDiscretized )

	statesIdx = statesIdx(:);
	[dummy,cols] = size( trajectory );
    nx= cols - 1;
	[nPlots,dummy] = size( statesIdx );

	if ( nx > 12 )
		useSubplots = 1;
	end

	if ( useSubplots == 0 )
		for i=1:nPlots
			plotFigure( trajectory,statesIdx(i),useSubplots,plotType,isDiscretized );
		end
	else
		numOfSubplots = 2*3;
		for i=1:numOfSubplots:nPlots
			plotIdx = statesIdx(i:min([i+numOfSubplots-1,nPlots]));
			plotFigure( trajectory,plotIdx,useSubplots,plotType,isDiscretized );
		end
	end

end



function [ ] = plotFigure( trajectory,plotIdx,useSubplots,plotType,isDiscretized )

	figure;

	for i=1:length(plotIdx)
		if ( useSubplots == 1 )
			%% multiple plots
			subplot(2,3,i);
			hold on;
		end

		plotHandle = plot( trajectory(:,1),trajectory(:,i+1) );
		set(plotHandle,'LineWidth',2);

		switch plotType
			case 0
				set(plotHandle,'Color','blue');
				legendY = [ 'Differential State ',num2str(plotIdx(i)) ];

			case 1
				set(plotHandle,'Color','red');
				legendY = [ 'Algebraic State ',num2str(plotIdx(i)) ];
		end

		switch isDiscretized
			case 0
				%% nothing to do

			case 1
				set(plotHandle,'LineStyle','--');
				set(plotHandle,'Marker','s');
		end

		ylabel( legendY,'FontSize',14 );
	end

end
