add_maggie_test(oni2imgs.exe
    SOURCES     oni2imgs.cpp oni2imgs.h database_player.h
    LINK        OpenNI
)

add_maggie_test(dgaitdb.exe
    SOURCES     dgaitdb.cpp dgaitdb.h database_player.h
    LINK        OpenNI disjoint_sets2 string_utils
)

add_maggie_test(liris2imgs.exe
    SOURCES     liris2imgs.cpp liris2imgs.h database_player.h
    LINK        xml_document
)

add_maggie_test(g3d2imgs.exe
    SOURCES     g3d2imgs.cpp g3d2imgs.h database_player.h
    LINK        xml_document
)
