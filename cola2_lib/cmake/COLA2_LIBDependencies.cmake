# Boost
find_package(Boost REQUIRED thread date_time filesystem)
include_directories(${Boost_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

# Tiny XML
find_package(TinyXML REQUIRED)

# Python 2
find_program(PYTHON "python")

# Python 3
find_program(PYTHON3 "python3")
