function [rho, order] = rankCorr(X, y, names)
% RANKCORR  Spearman rank correlation, without the Statistics Toolbox.
%
%   [rho, order] = rankCorr(X, y, names)
%   X  n-by-p matrix of sampled parameters
%   y  n-by-1 response
%
% Spearman rather than Pearson because the relationships here are monotonic but
% emphatically not linear -- cross-track error blows up nonlinearly as yaw
% damping falls -- and because several parameters are sampled log-uniformly.
% Rank correlation measures "does more of this make it worse" without assuming
% a functional form.
%
% tiedrank and corr both live in the Statistics Toolbox, which this repo does
% not require, so both are implemented here.

if nargin < 3, names = arrayfun(@(k) sprintf('p%d',k), 1:size(X,2), 'uni', 0); end

y = y(:);
ok = isfinite(y) & all(isfinite(X), 2);
X = X(ok,:);  y = y(ok);
p = size(X,2);

ry = tiedRankLocal(y);
rho = nan(1,p);
for k = 1:p
    rx = tiedRankLocal(X(:,k));
    rho(k) = pearson(rx, ry);
end

[~, order] = sort(abs(rho), 'descend');

if nargout == 0
    for k = order
        fprintf('  %-18s rho = %+.3f\n', names{k}, rho(k));
    end
end
end

% -------------------------------------------------------------------------
function r = tiedRankLocal(x)
% TIEDRANKLOCAL  Ranks with ties averaged.
n = numel(x);
[xs, i] = sort(x(:));
r = zeros(n,1);
r(i) = 1:n;
% Average the ranks within each run of equal values.
k = 1;
while k <= n
    j = k;
    while j < n && xs(j+1) == xs(k), j = j + 1; end
    if j > k
        r(i(k:j)) = mean(k:j);
    end
    k = j + 1;
end
end

% -------------------------------------------------------------------------
function c = pearson(a, b)
a = a(:) - mean(a);  b = b(:) - mean(b);
d = sqrt(sum(a.^2) * sum(b.^2));
if d == 0, c = 0; else, c = sum(a.*b) / d; end
end
