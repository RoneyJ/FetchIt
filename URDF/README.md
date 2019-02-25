Here is a simulated workspace for Fetch robot. The Fetch.stl file is in .zip since it was too large to upload on to github, so to use it please unzip it after clone the contents.

INSTRUCTIONS BRFORE YOU LAUNCH!
  Please copy and paste all the folders in Fetch_parts to .gazebo/models. .gazebo can be found when you type ctrl+h at home folder. If you don't complete this step, nothing will show up in your gazebo!!! Ask me on slack if you couldn't figure out how. There is a image file shows how the space should look like. 
  
COLLISION MODELS:
  The collision for each parts are pretty much the same as the visual model, however the kit only has collision on the bottom. So make sure don't flip the kit upside down otherwise it will face into the ground. I will fix this issue by adding another link on the handle. 

To launch the workspace, simply type `roslaunch worlds Fetch_kit.launch`.
Make sure to use Gazebo 9 and melodic since thats the requirements for the competition.


Bruce
