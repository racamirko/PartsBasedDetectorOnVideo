PartsBasedDetectorOnVideo
=========================

Just serial processing of video with PartsBasedDetector.


Model creation
--------------

I had a bit of a problem figuring out this part, so here goes the recipe:

 1. Train your part-based model in matlab (code available from Ramanan's site)
 2. Convert it to 'Pose' format with PartBasedDetector/matlab/modelTransfer.m
 3. use ModelTransfer executable in PartBasedDetector to transform it from .mat -> .xml format
 4. enjoy using the PartBasedDetectorOnVideo which can now accept your trained model

Command line options
--------------------

```
Program parameters:
  -h [ --help ]         produce help message
  -m [ --model ] arg    Model file to use. xml, mat or jaml format
  -v [ --video ] arg    Video file to analyse
  -d [ --dir ] arg      Output folder
  -r [ --resume ]       Resume option [default: false]
  -n [ --nms ] arg (=0) NMS filter threshold, percentage in the range 0.0-1.0, 
                        default O.0
  --mirror              Mirroring option - mirror the video image horizontally 
                        and process 2x (with corrections of the detections) 
                        [default: false]
  -s [ --size ] arg     Size filter. Eliminate all instances bigger then 
                        [width],[height]
```


Referenced code
---------------

Project uses the code developed by hbristow (cvmatio and PartsBasedDetector), based on the research of Deva Ramanan and friends.

Used libraries
--------------

 * PartsBasedDetector (available on github as my or hbristow repository)
 * OpenCV 2.4.6 (latest at the moment)
 * glog - google logging library
 * Boost::filesystem, system, program options
 * ncurses - for a nicer progress report

Also, OpenCV as always.

Contact
-------
name.lastname@epfl.ch
