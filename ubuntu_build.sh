rm -rf ~/.hunter
rm -rf build/*
cmake -H. -Bbuild -D BUILD_SHARED_LIBS=ON
cmake --build build -j70
