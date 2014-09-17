# Copyright (C) 2007 by Cristóbal Carnero Liñán
# grendel.ccl@gmail.com
#
# This file is part of blob.
#
# blob is free software: you can redistribute it and/or modify
# it under the terms of the Lesser GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# blob is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# Lesser GNU General Public License for more details.
#
# You should have received a copy of the Lesser GNU General Public License
# along with blob.  If not, see <http://www.gnu.org/licenses/>.
#

find_package(OpenCV REQUIRED)

set(blob_blob Blob.cpp BlobResult.cpp BlobExtraction.cpp)

set_source_files_properties(${blob_SRC}
                            PROPERTIES
                            COMPILE_FLAGS "-O3"
)

add_library(blob SHARED ${blob_blob})
