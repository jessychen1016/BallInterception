mkdir build
cd build
cmake -A x64 -MAKE_BUILD_TYPE=Release ..
cmake --build . -- /p:Configuration=Release /maxcpucount:8 /p:CL_MPCount=8
cd ..