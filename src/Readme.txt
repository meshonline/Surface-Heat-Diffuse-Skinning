1. Install GNU gcc and g++ with static lib:

Windows:
Download mingw-w64-install.exe, select the posix thread library and the x64 platform to install.

CentOS:
yum group install "Development Tools"
yum install glibc-static libstdc++-static

macOS:
brew install gcc
brew link —overwrite gcc

2. How to build:

Windows:
From mingw-w64's terminal window:
g++ *.cpp -static-libgcc -static-libstdc++ -std=c++11 -Wl,-Bstatic -lstdc++ -lpthread -Wl,-Bdynamic -O2 -ftree-vectorize -fopt-info-vec -o shd

Linux:
From terminal window:
g++ *.cpp -static-libgcc -static-libstdc++ -std=c++11 -pthread -O2 -ftree-vectorize -fopt-info-vec -o shd

macOS:
From terminal window:
g++-7 *.cpp -static-libgcc -static-libstdc++ -std=c++11 -pthread -O2 -ftree-vectorize -fopt-info-vec -o shd
