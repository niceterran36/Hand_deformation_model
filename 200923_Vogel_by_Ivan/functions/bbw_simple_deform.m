function [new_points] = bbw_simple_deform(varargin)
  % SIMPLE_DEFORM Deform a mesh (V,F) using linear blend skinning or dual quaternion
  % skinning, by applying transformations at a set of control points C and
  % propogating the deformation to the mesh via correspondence weights W.
  %
  % simple_deform(V,F,C,W,...)
  %
  % Inputs:
  %  V  #V by 2 list of vertex positions
  %  F  #F by 3 list of face indices
  %  C  #C by 2 list of control point positions
  %  W  weights, #vertices by #handles matrix of weights
  % Output:
  %   gid  index into global variable g_Deform, which gives access to plot
  %     handles and input variables
  %
  % Global Output:
  %   struct array g_Deform:
  %     g_Deform(gid).R  pose rotations of point handles
  %     g_Deform(gid).new_C  pose positions of control vertices
  %     g_Deform(gid).update_positions  function handle to update positions
  %       based on fields R and new_C
  %     g_Deform(gid).tsh  plot handle to main triangle mesh plot
  %     g_Deform(gid).wvsh  plot handle to weight visualization plot
  % 
  % Copyright 2011, Alec Jacobson (jacobson@inf.ethz.ch)
  %
  % See also: deform
  %

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % parse input
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % vertex positions of mesh
  V = varargin{1};
  % face indices of mesh
  F = varargin{2};
  % control vertices of skeleton
  C = varargin{3};
  % deformation weights
  W = varargin{4};
  % moving distance
  D = varargin{5};

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Set default parameters
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % set default point handles
  P = 1:size(C,1);
  % Be sure that control vertices are in 2D
%   if(size(C,2) == 3)
%     C = C(:,1:2);
%   end
  BE = [];
  new_points = [];

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Prepare output
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  global g_Deform;
  gid = numel(g_Deform)+1;
  % keep track of control positions at mouse down
  g_Deform(gid).new_C = [];
  % keep track of rotations stored at each control point, for 2D this is a m
  g_Deform(gid).update_positions = @update_positions;

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Set up plots
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % Set up figure with enough subplots for any additional visualizations
  % clear current figure
%   close 99

  % plot the original mesh
  figure(99);
  g_Deform(gid).tsh = trisurf(F,V(:,1),V(:,2),V(:,3), 'FaceColor',[1 0.88 0.77], 'EdgeColor', 'none');
  view(2);
  axis equal
  axis manual
  light('Position', [3 5 7], 'Style', 'infinite');
  lighting gouraud;
  material dull;
  hold on;

  % plot the control points (use 3D plot and fake a depth offset by pushing
  % control points up in z-direction)
  C_plot = scatter3( ...
    C(:,1),C(:,2),C(:,3), ... 
    'o','MarkerFaceColor',[1 0 0], 'MarkerEdgeColor','k', 'LineWidth',2,'SizeData',100);
  hold off;
  
  g_Deform(gid).new_C = [get(C_plot,'XData')' get(C_plot,'YData')' get(C_plot, 'ZData')'];
  for i = 1:size(D, 1)
      g_Deform(gid).new_C(i,1) = g_Deform(gid).new_C(i,1)+D(i, 1);
      g_Deform(gid).new_C(i,2) = g_Deform(gid).new_C(i,2)+D(i ,2);
      g_Deform(gid).new_C(i,3) = g_Deform(gid).new_C(i,3)+D(i, 3);
  end
  update_positions();
  close
  return

  function update_positions()
    % update display positions
    set(C_plot,'XData',g_Deform(gid).new_C(:,1));
    set(C_plot,'YData',g_Deform(gid).new_C(:,2));
    set(C_plot,'ZData',g_Deform(gid).new_C(:,3));

    % USING LINEAR BLEND SKINNING
    % get transformations stored at each point and bone handle
    TR = bbw_skinning_transformations(C,P,BE,g_Deform(gid).new_C);
    % linear blend skinning
%     [new_points] = lbs(V(:,1:2),TR,W);
    [new_points] = bbw_lbs(V,TR,W);

    % update mesh positions
    set(g_Deform(gid).tsh,'Vertices',new_points);
  end
end