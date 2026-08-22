# Copyright 2026 Alexandros Filotheou
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

find_package(PkgConfig REQUIRED)

if(NOT TARGET PkgConfig::FFTW3)
  pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3)
endif()

if(NOT TARGET PkgConfig::CSM)
  pkg_check_modules(CSM REQUIRED IMPORTED_TARGET csm)

  # Refused for the reason cbgl's own CMakeLists.txt gives at length: csm is
  # LGPL v3, it is linked dynamically to satisfy that licence, and a csm built
  # without BUILD_SHARED_LIBS resolves to an archive with nothing said.
  if(CSM_LINK_LIBRARIES MATCHES "\\.a(;|$)")
    message(FATAL_ERROR
      "csm resolved to a static archive (${CSM_LINK_LIBRARIES}). cbgl links "
      "csm dynamically because csm is LGPL v3 and a static link does not "
      "satisfy that licence's relinking requirement. Rebuild csm with "
      "-DBUILD_SHARED_LIBS=ON.")
  endif()
endif()

if(NOT TARGET yaml-cpp::yaml-cpp)
  find_package(yaml-cpp REQUIRED)
endif()
