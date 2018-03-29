1. Install GNU gcc and g++ with static lib:

Windows:
Download mingw-w64-install.exe, double click to install.
From launch start, run mingw-w64's terminal window.

CentOS:
yum group install "Development Tools"
yum install glibc-static libstdc++-static

macOS:
brew install gcc
brew link —overwrite gcc

2. How to build:

Windows:
g++ *.cpp -static-libgcc -static-libstdc++ -std=c++11 -Wl,-Bstatic -lstdc++ -lpthread -Wl,-Bdynamic -O2 -ftree-vectorize -fopt-info-vec -o shd

Linux:
g++ *.cpp -static-libgcc -static-libstdc++ -std=c++11 -pthread -O2 -ftree-vectorize -fopt-info-vec -o shd

macOS:
g++-7 *.cpp -static-libgcc -static-libstdc++ -std=c++11 -pthread -O2 -ftree-vectorize -fopt-info-vec -o shd
