Plan out the development

0. Image directories 

1. Image reader that's done in the main

2. tracker 

3. image feature track class 


How to evaluate the pipeline 

Benchmark 

Admit it.
You are just a beginner still


Where is the good place for feature extraction

It's the Frame class. This class gets the input of image. 


How do I experiment the different types of features 

Class inheritance 

-> Feature extractor : generalized feature extractor 
-> base class 
-> inherit 


Optical flow approach from prev to current image tracking
no compact match vector 
-> each index in the list directly corresponds to the feature idx. 

match idx 
it's not the match pair that's obtained. 

Each corresponding feature is in the same idx as in the previous tracking frame's feature sets


With descriptor matching approach
current to the previous tracking



Why?



Match against only the previous frame?
Or match against other frames as well

Start with just one frame first


Fundamental matrix and epipolar check
How expensive are they?

I am using it as a check for the good match verification.
Is it valid?

Goals
1. Outliers 
Want to compare how different approaches produce robust tracking
circulr check library


2. Speed 


Optical flow + feature descriptor + circular check 


Monocular -> scale
Retrieve the first scale 


Opportunity
If I can make my own library for this complex system, there would be different things I can tweak to make each experiment into some sorts of research ideas.


AI class 


Feature extractor can be in the frame class 

smart pointer 

Tracker -> contains the circular frames that it's tracking

Map Class -> need to contain the active frames 
Mapper
 Triangulator -> for the scale, just be consistent 
Optimizer

Frame, Timestamp, 


monocular tracking -> if tracking gets lost, scale cannot be recovered
At the initializer, use depth to recover the scale 

a. frame generation
b. feature extraction 
c. tracking
d. map generation
e. optimization

Development plan 1

1. Tracking Features between frames and store the tracked information

2. Tracking improvement by buffer, windowed block matching, combining optical flow with descriptor

3. Triangulator for stable features 
First by 2 frames 

4. in the optimizer of the sliding windowed fashion, optimize the Map and the Camera poses 

Devel plan 2 
1. Initializer using the depth frame for scale consistency and initial pose

2. Use stereo camera pair

3. generic calibration
4.  

Evaluation criteria
1. Tracking lost 
2. depth association with the GT data (from other mature slam solution) and computing the accuracy
3. Frame pose accuracy 



