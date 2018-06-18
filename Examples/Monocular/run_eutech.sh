cd ../../build
make -j4
cd ../Examples/Monocular
#./euro_imu ../../Vocabulary/ORBvoc.bin ../../config/euroc.yaml /home/aayush/workspace/sandbox/mav0/cam0/data ../../../../mav0/cam0/data.csv ../../../../mav0/imu0/data.csv
./eutech_imu ../../Vocabulary/ORBvoc.bin ../../config/mobius_down.yaml /home/aayush/workspace/sandbox/quad-nav/videos/2017-01-22_07:59:41.mkv /home/aayush/workspace/sandbox/quad-nav/videos/2017-01-22_07:59:41.log /home/aayush/workspace/sandbox/quad-nav/videos/


#./eutech_imu ../../Vocabulary/ORBvoc.bin ../../config/mobius.yaml /home/aayush/workspace/sandbox/quad-nav/videos/2017-04-25_06_11_56.mkv /home/aayush/workspace/sandbox/quad-nav/videos/2017-04-25_06_11_56.log
