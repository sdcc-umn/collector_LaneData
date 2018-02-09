For data collection:

	Start the bot and camera: roslaunch practice_xb turtle_astra_ar.launch

Make sure to make new folder each run (otherwise it will replace the old values)
so create /home/turtlebot/datasets/trial_n/
and and images folder inside that directory (/home/turtlebot/datasets/trial_n/images)
Then start the data collection node: rosrun practice_xb go_.py --dir /home/turtlebot/datasets/<trial_foldername>/
The images will be saved in the images folder and annotation.txt will contain the navigation commands

<<< Press f to go forward, r and l to rotate >>>
when done, press q

Make a short demo run to check the data before going for a long data collection process




FYI:
For the astra camera of turtle bot:
rosrun camera_calibration cameracalibrator.py --size 9x11 --square 0.02 image:=/camera/rgb/image_raw camera:=/camera/rgb
