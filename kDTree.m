classdef kDTree < handle
    %% Constructor
    properties(SetAccess=immutable,GetAccess=protected)
        Elements    (1,1)
        Dimensions	(1,1)
    end
    properties(SetAccess=immutable)
        RootIndex	(1,1)
    end
    properties(SetAccess=protected)
        LeftIndices	(:,1)
        RightIndices(:,1)
    end
    methods
        function a_tree = kDTree(pdat)
            assert(ismatrix(pdat),'Position data must be a 2D matrix!');
            % init size
            index_seed          = kDTree.get_index_seed(pdat);
            [   a_tree.Elements,    ...
            	a_tree.Dimensions	...
                ]               = deal(zeros('like',index_seed));
            % get size of position data
            if      isempty(pdat)
                % do nothing
            else
                [   a_tree.Elements(1),	...
                    a_tree.Dimensions(1)...
                    ]           = size(pdat);
            end
            % init indices
            a_tree.RootIndex	= zeros('like',index_seed);
            [   a_tree.LeftIndices,     ...
                a_tree.RightIndices     ...
                ]               = deal(zeros(a_tree.Elements,1,'like',index_seed));
            % shortcut
            if      isempty(pdat)
                return;
            end
            
            a_tree.RootIndex(1) =       ...
                get_root_index(         ...
                a_tree,                 ...
                a_tree.Dimensions,      ...
                (1:a_tree.Elements).',	...
                pdat                        );
        end
    end
    methods(Access=protected)
        function mindex = get_root_index(a_tree,cprevx,sindex,pdat)
            cindex                          = 1+mod(cprevx,a_tree.Dimensions);
            [mindex,lindex,ldat,rindex,rdat]= kDTree.parse_data(cindex,sindex,pdat);
            if      isempty(lindex)
                % do nothing
            elseif	isscalar(lindex)
                a_tree.LeftIndices(mindex)	= lindex;
            else
                a_tree.LeftIndices(mindex)  = get_root_index(a_tree,cindex,lindex,ldat);
            end
            
            if      isempty(rindex)
                % do nothing
            elseif	isscalar(rindex)
                a_tree.RightIndices(mindex)	= rindex;
            else
                a_tree.RightIndices(mindex)	= get_root_index(a_tree,cindex,rindex,rdat);
            end
        end
    end
    methods(Static,Access=protected)
        function index_seed = get_index_seed(pdat)
            % This allows for the creation of an array in the same data
            % location as pdat. 
            empty_array	= pdat([]);
            bool_seed	= empty_array==empty_array;
            index_seed  = double(bool_seed);
        end
        function [mindex,lindex,ldat,rindex,rdat] = parse_data(cindex,sindex,pdat)
            [mid_relative,lhs_bools,rhs_bools] = kDTree.parse_indices(cindex,pdat);
            
            mindex	= sindex(mid_relative);
            lindex  = sindex(lhs_bools);
            ldat    = pdat(lhs_bools,:);
            rindex  = sindex(rhs_bools);
            rdat    = pdat(rhs_bools,:);
            
        end
        function [mid_relative,lhs_bools,rhs_bools] = parse_indices(cindex,pdat)
            axis_dat    = pdat(:,cindex);
            median_axis	= median(axis_dat);
            lhs_bools   = median_axis>=axis_dat;
            rhs_bools   = ~lhs_bools;
            
            % remove middle from lhs
            mid_relative    = find(median_axis==axis_dat,1,'first');
            if      isempty(mid_relative)
                median_axis	= max(axis_dat(lhs_bools));
                mid_relative= find(median_axis==axis_dat,1,'first');
            end
            lhs_bools(mid_relative)	= false;
            
        end
    end
end