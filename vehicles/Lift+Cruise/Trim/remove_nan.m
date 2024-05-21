function XEQ = remove_nan(XEQ)
  % removes trim points with NaN from all trim tables

  [N,M,L,K] = size(XEQ);

  RMV_COL = [];

  for ll = 1:L
    for kk = 1:K
      xeq = XEQ(:,:,ll,kk);
      [row, col] = find(isnan(xeq));
      RMV_COL = [RMV_COL; col(1:N:end)];
    end
  end

  XEQ(:,RMV_COL,:,:) = [];
