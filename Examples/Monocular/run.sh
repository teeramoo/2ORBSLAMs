cd ../../build
make -j4
cd ../Examples/Monocular

#./euro_imu ../../Vocabulary/ORBvoc.bin ../../config/euroc.yaml /home/aayush/workspace/sandbox/mav0/cam0/data ../../../../mav0/cam0/data.csv ../../../../mav0/imu0/data.csv
./euro_imu ../../Vocabulary/ORBvoc.bin ../../config/euroc.yaml /home/aayush/workspace/sandbox/V1_02_medium/mav0/cam0/data ../../../../V1_02_medium/mav0/cam0/data.csv ../../../../V1_02_medium/mav0/imu0/data.csv /home/aayush/workspace/sandbox/V1_02_medium/mav0/ 0
