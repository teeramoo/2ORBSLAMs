cd ../../build
make -j4
cd ../Examples/Monocular

RUN_DIR='2017-05-30:11:16:22'
OUT_DIR=/home/aayush/workspace/projects/watcher/results/$RUN_DIR/

#'2017-05-11:13:48:26'
#'2017-05-11:11:45:43'
#'2017-05-11:13:43:16' #Good loop on small trees


./eutech_vertical ../../Vocabulary/ORBvoc.bin ../../config/sony.yaml /home/aayush/Videos/MAH00014.MP4 /home/aayush/workspace/projects/record_run/output/2017-05-11:13:34:19/outputIMU $OUT_DIR 150

