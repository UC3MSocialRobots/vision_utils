
IF(AD_USE_OPENCV)
    add_maggie_lib (OcrUtils
        SOURCES     OcrUtils.cpp
        LINK         string_utils opencv # tesseract_full
    )
ENDIF(AD_USE_OPENCV)
