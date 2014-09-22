add_maggie_test(pplp_benchmarker.exe
    SOURCES     pplp_benchmarker.cpp pplp_benchmarker.h
    LINK        lap
)

add_maggie_test(dgait2ppl.exe
    SOURCES     dgait2ppl.cpp
    LINK        disjoint_sets2
)

add_maggie_test(KTP2PPL.exe
    SOURCES     KTP2PPL.cpp
    LINK        disjoint_sets2
)
