function S = sampleUncertainty(mode, n, seed)
% SAMPLEUNCERTAINTY  Draw values for every parameter defined in uncertainty.m.
%
%   S = sampleUncertainty('nominal')         -> 1 struct, all .nom values
%   S = sampleUncertainty('lhs', n, seed)    -> 1xn struct array, Latin hypercube
%   S = sampleUncertainty('corners')         -> 1x2 struct array, all-lo & all-hi
%
% LHS is used instead of plain Monte Carlo so that 500 runs actually cover the
% 15-dimensional box; plain random sampling leaves large voids at that count.
% The LHS here is hand-rolled (stratify each margin, permute independently) so
% no Statistics Toolbox is required.

if nargin < 1, mode = 'nominal'; end
if nargin < 2, n = 1; end
if nargin < 3, seed = 0; end

U = uncertainty();
f = fieldnames(U);
nf = numel(f);

switch lower(mode)
    case 'nominal'
        for k = 1:nf, S.(f{k}) = U.(f{k}).nom; end

    case 'corners'
        for k = 1:nf
            S(1).(f{k}) = U.(f{k}).lo;
            S(2).(f{k}) = U.(f{k}).hi;
        end

    case 'lhs'
        rng(seed, 'twister');
        % Stratified unit samples: one per bin, jittered inside the bin, then
        % each column permuted independently so the marginals stay uniform but
        % the joint fill is far better than i.i.d. sampling.
        Q = zeros(n, nf);
        for k = 1:nf
            bins = ((1:n)' - 1 + rand(n,1)) / n;
            Q(:,k) = bins(randperm(n));
        end
        for i = 1:n
            for k = 1:nf
                S(i).(f{k}) = mapQuantile(U.(f{k}), Q(i,k));
            end
        end

    otherwise
        error('sampleUncertainty: unknown mode "%s"', mode);
end
end

% -------------------------------------------------------------------------
function v = mapQuantile(e, q)
% MAPQUANTILE  Inverse-CDF map from a unit quantile q to the parameter range.
switch e.dist
    case 'uniform'
        v = e.lo + q * (e.hi - e.lo);
    case 'logunif'
        v = exp(log(e.lo) + q * (log(e.hi) - log(e.lo)));
    otherwise
        error('mapQuantile: unknown distribution "%s"', e.dist);
end
end
