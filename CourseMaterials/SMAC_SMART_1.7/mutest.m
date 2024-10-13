% this routine checks whether the mu upper bound is less than 1. if yes,
% ubnd is set to 1, the algorithm stops and wctest,tabwc,wcsen,scal are
% left empty. if no, wctest is set to the frequency at which the mu-test
% failed and tabwc to the list of unvalidated frequency intervals. then,
% if the mu-sensitivities are not used to cut the domain, the algorithm
% stops, ubnd is set to the current value of the mu upper bound (strictly
% larger than 1), and wcsen,scal are left empty. otherwise, the mu upper
% bound computation goes on until the whole frequency range is validated.
% in this case, when the algorithm finally stops, ubnd is a guaranteed mu
% upper bound, while wcsen contains the frequency at which this bound was
% obtained and scal the associated LMI-based D,G scalings.
