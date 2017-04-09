# Flyspy3
Flyspy3 is a software for 3D tracking Drosophila fly activities in a controlled environment using multiple synchronized cameras. For more description of the method please see our paper [Three-dimensional tracking and behaviour monitoring of multiple fruit flies](https://www.ncbi.nlm.nih.gov/pubmed/23034355).
# How to compile
Open the solution file in Visual Studio and compile the code in x64 settings. The code is dependent to the OpenCV library, but all of the dependencies are already included, so the code should compile without any extra setting.

# How to run
**flyspy3.exe** *< number of targets>* *< number of views>* *< input method>* *< _tracking method_>* *< path to the first input AVI file>* *< path to the projection matrix file>* *< path to the projection matrix file>* *< prefix for all output files>* *< first frame to process>* *< last frame to process>*


Here is a short description from arguments that are not clear:

| Arguement        | Description           |
| ------------- |:-------------:|
| Path to the first input AVI file| FlySpy3 assumes AVI files have the same file name with a number at the end (View2.avi, View3.avi, ..|
| input method      | AVI or CENTROID (choose centroid if you have a text file with 2D locations of the targets     | 
| tracking method | HUNGARIAN or DFS or STABLE_POLYAMORY, Hungarian is recommended   | 

# Sample data
You can download a sample dataset which includes 4 AVI files and one text file with calibration information from [here](https://drive.google.com/drive/folders/0Bzem1LPfAlcOR2F4S1F1dnRBaWM). (~1.7GB).
# Output sample
Here is a sample output from a 3D tracking experiment with multiple targets.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=i_FFQGsIDJc
" target="_blank"><img src="http://img.youtube.com/vi/i_FFQGsIDJc/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>
