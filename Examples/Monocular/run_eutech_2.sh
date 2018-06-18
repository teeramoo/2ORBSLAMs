cd ../../build
make -j4
cd ../Examples/Monocular

#RUN_DIR='2017-05-30:11:16:22'
RUN_DIR='test'
OUT_DIR=/home/aayush/workspace/projects/watcher/results/$RUN_DIR/

mkdir -p $OUT_DIR

#'2017-05-11:13:48:26'
#'2017-05-11:11:45:43'
#'2017-05-11:13:43:16' #Good loop on small trees

#./eutech_imu_2 ../../Vocabulary/ORBvoc.bin ../../config/sony.yaml /home/aayush/workspace/projects/record_run/output/2017-05-11:11:45:43/outputVid.mkv /home/aayush/workspace/projects/record_run/output/2017-05-11:11:45:43/outputVid.txt /home/aayush/workspace/projects/record_run/output/2017-05-11:11:45:43/outputIMU

#./eutech_imu_2 ../../Vocabulary/ORBvoc.bin ../../config/sony.yaml /home/aayush/workspace/projects/record_run/output/$RUN_DIR/outputVid.mkv /home/aayush/workspace/projects/record_run/output/$RUN_DIR/outputVid.txt /home/aayush/workspace/projects/record_run/output/$RUN_DIR/outputIMU $OUT_DIR

#./eutech_imu ../../Vocabulary/ORBvoc.bin ../../config/mobius.yaml /home/aayush/workspace/sandbox/quad-nav/videos/2017-04-25_06_11_56.mkv /home/aayush/workspace/sandbox/quad-nav/videos/2017-04-25_06_11_56.log

#set -e 
#cd $OUT_DIR

#./../../../Colorize_Cloud/build/colorizeCloud MapPoints_0.pcd KeyFrameTrajectory_0.txt
#echo "Color done"
#./../../../cloud_visualizer/build/cloudVisualizer -r colored_Combined.pcd colored_KeyFrame.pcd
#echo "Scaling done"


# Mobius
#./eutech_imu_2 ../../Vocabulary/ORBvoc.bin ../../config/mobius.yaml /home/aayush/workspace/projects/record_run/LaserOutput/$RUN_DIR/outputVid.mkv /home/aayush/workspace/projects/record_run/LaserOutput/$RUN_DIR/outputVid.txt /home/aayush/workspace/projects/record_run/LaserOutput/$RUN_DIR/outputIMU $OUT_DIR

./eutech_imu_2 ../../Vocabulary/ORBvoc.bin ../../config/mobius.yaml /home/aayush/workspace/demo/farm10_1499057896633/outputVidH.mkv /home/aayush/workspace/demo/farm10_1499057896633/frameData.txt /home/aayush/workspace/demo/farm10_1499057896633/outputIMU $OUT_DIR
