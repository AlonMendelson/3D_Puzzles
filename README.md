# 3D_Puzzles
Roy Kinamon & Alon Mendelson\
This is a git repository of our final project for the course Algorithms for Modeling, Fabrication and Printing of 3D Objects.\
We designed & implemented an algorithm that turns a 3D model into pieces of puzzle that can be assembled together to the original model.
## Running the code
### Prerequisites
* python ≥ 3.5
* OpenSCAD (make sure it is added to the path variable)
### Command line example
```
python main.py Bunny outputs
```
This creates a puzzle from the Bunny.stl file from the models folder and saves the results to folder outputs within the repository\
Note: runtime can be several minutes since boolean operations are slow
## Extra materials
See the Results folder for stl files of puzzle pieces for different models as well as pictures of assembled puzzles that we printed
