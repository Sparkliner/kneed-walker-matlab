function cost = costFun(x,gval,gamval,nolockdat,u)
  x = x(1:nolockdat.numlinks*2,:);
  for I=1:size(x,2)
      th = num2cell(x(1:nolockdat.numlinks,I));
      th_d = num2cell(x(nolockdat.numlinks+1:end,I));
      T = nolockdat.Kf(th{:},th_d{:});
      V = nolockdat.Pf(gval,gamval,th{:});
      cost(:,I) = (T-V).^2;
  end
%cost = sum(u.*u,1);
end