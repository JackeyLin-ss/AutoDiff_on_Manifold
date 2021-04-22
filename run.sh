mkdir build
cd build && cmake ..
make -j 
./pose_estimation_test ../data/local_map.txt ../data/feature_and_map_points.txt
