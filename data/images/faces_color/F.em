#~ A maximization step block of an .em file starts with a single line describing
#~ the number of shared parameters blocks that will follow. Then, each shared
#~ parameters block follows, in the format described in the next subsection.
#~
#~ A shared parameters block of an .em file starts with a single line consisting
#~ of the name of a ParameterEstimation subclass and its parameters in the
#~ format of a PropertySet. For example:
#~
#~ CondProbEstimation [target_dim=2,total_dim=4,pseudo_count=1]
#~
#~ The next line contains the number of factors that share their parameters.
#~
#~ Then, each of these factors is specified on separate lines (possibly
#~ seperated by empty lines), where each line consists of several fields
#~ seperated by a space or a tab character.
#~
#~ The first field contains the index of the factor in the factor graph.
#~
#~ The following fields should contain the variable labels of the variables on
#~ which that factor depends, in a specific ordering. This ordering can be
#~ different from the canonical ordering of the variables used internally in
#~ libDAI (which would be sorted ascendingly according to the variable labels).
#~
#~ The ordering of the variables specifies the implicit ordering of the shared
#~ parameters: when iterating over all shared parameters, the corresponding
#~ index of the first variable changes fastest (in the inner loop), and the
#~ corresponding index of the last variable changes slowest (in the outer loop).
#~ By choosing the right ordering, it is possible to let different factors (
#~ depending on different variables) share parameters in parameter learning
#~ using EM. This convention is similar to the convention used in factor blocks
#~ in a factor graph .fg file (see Factor block format).
1

5
# block V
CondProbEstimation [target_dim=2,total_dim=2,pseudo_count=1]
1
0 0
# block V, F
CondProbEstimation [target_dim=3,total_dim=6,pseudo_count=1]
1
1 1 0
# block V, S
CondProbEstimation [target_dim=10,total_dim=20,pseudo_count=1]
1
2 2 0
# block V, T
CondProbEstimation [target_dim=10,total_dim=20,pseudo_count=1]
1
3 3 0
# block V, F, N
CondProbEstimation [target_dim=2,total_dim=12,pseudo_count=1]
1
4 4 1 0
