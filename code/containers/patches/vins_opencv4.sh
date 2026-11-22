#!/usr/bin/env bash
# Адаптация VINS-Fusion (написан под OpenCV 3) к OpenCV 4 из Ubuntu 20.04.
set -euo pipefail
ROOT="$1"
cd "$ROOT"

# старые C-константы OpenCV -> C++ API
find . -type f \( -name '*.cpp' -o -name '*.h' \) -print0 | xargs -0 sed -i \
    -e 's/CV_LOAD_IMAGE_GRAYSCALE/cv::IMREAD_GRAYSCALE/g' \
    -e 's/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g' \
    -e 's/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g' \
    -e 's/CV_RGB2GRAY/cv::COLOR_RGB2GRAY/g' \
    -e 's/CV_GRAY2RGB/cv::COLOR_GRAY2RGB/g' \
    -e 's/CV_GRAY2BGR/cv::COLOR_GRAY2BGR/g' \
    -e 's/CV_FONT_HERSHEY_SIMPLEX/cv::FONT_HERSHEY_SIMPLEX/g' \
    -e 's/CV_AA/cv::LINE_AA/g' \
    -e 's/CV_CALIB_CB_ADAPTIVE_THRESH/cv::CALIB_CB_ADAPTIVE_THRESH/g' \
    -e 's/CV_CALIB_CB_NORMALIZE_IMAGE/cv::CALIB_CB_NORMALIZE_IMAGE/g' \
    -e 's/CV_CALIB_CB_FILTER_QUADS/cv::CALIB_CB_FILTER_QUADS/g' \
    -e 's/CV_CALIB_CB_FAST_CHECK/cv::CALIB_CB_FAST_CHECK/g' \
    -e 's/CV_ADAPTIVE_THRESH_MEAN_C/cv::ADAPTIVE_THRESH_MEAN_C/g' \
    -e 's/CV_THRESH_BINARY/cv::THRESH_BINARY/g' \
    -e 's/CV_SHAPE_CROSS/cv::MORPH_CROSS/g' \
    -e 's/CV_SHAPE_RECT/cv::MORPH_RECT/g' \
    -e 's/CV_RETR_CCOMP/cv::RETR_CCOMP/g' \
    -e 's/CV_CHAIN_APPROX_SIMPLE/cv::CHAIN_APPROX_SIMPLE/g' \
    -e 's/CV_TERMCRIT_EPS/cv::TermCriteria::EPS/g' \
    -e 's/CV_TERMCRIT_ITER/cv::TermCriteria::MAX_ITER/g'

# Ceres из Ubuntu 20.04 и cv_bridge Noetic требуют C++14
find . -name CMakeLists.txt -print0 | xargs -0 sed -i \
    -e 's/-std=c++11/-std=c++14/g' \
    -e 's/CMAKE_CXX_STANDARD 11/CMAKE_CXX_STANDARD 14/g'

# константы из C-API, которые остались без C++-аналога в коде
for f in $(grep -rl --include='*.cc' --include='*.cpp' --include='*.h' 'opencv' .); do
    if grep -q 'CV_' "$f" && ! grep -q 'imgproc_c.h' "$f"; then
        sed -i '0,/#include/s//#include <opencv2\/imgproc\/imgproc_c.h>\n#include <opencv2\/calib3d\/calib3d_c.h>\n#include/' "$f"
    fi
done
