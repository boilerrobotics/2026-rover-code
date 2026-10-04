# 2026-rover-code
Rover Code Repository for URC 2026-2027 Season


# Structure
```text
2026-rover-code/
├── .devcontainer/
│   ├── devcontainer.json
│   ├── Dockerfile
├── src/
│   ├── your packages
├── bags/
│   ├── your bags
├── md_images/
│   ├── images rendered in this README
├── ros_odrive/
│   ├── odrive stuff
├── fetch_bags.sh
├── post_create.sh
├── .gitignore
└── README.md
```

Once you build and open in container, VSCode will clone the ODrive repository for you so that you can access the custom messages like ControlMessage and ControllerStatus. 

Bags contains bag files (explained later) (downloaded after container starts), .devcontainer contains the container configuration, src will contain the ROS stuff you guys are developing, ros_odrive contains the odrive messages like I mentioned earlier.

fetch_bags.sh is a shell script to fetch the bag files from the club's google drive (which is where I put them earlier) because Github wouldn't let me upload them as part of the repository. It will run whenever you start the container, but if you want to rerun it you can just run 
```bash
bash fetch_bags.sh
```
from root and it will fetch any new bag files. Keep in mind if the bag file name is the same it won't fetch it, even if the content is different. 

post_create.sh is the container post creation script that runs as soon as vscode finishes building the container. You don't need to worry about it for now.

# Development
When you want to develop, 
1. Switch to your branch. If you don't have one, make one and let your teammates know.
2. Make packages to group your nodes inside the src directory.
3. Write your code and test using the bag records.
    - build, source, run
    - to use the ODrive custom messages you need to build the ODrive packages and source inside    there first. You need to source that package in every terminal you open. 
4. Commit as you go.
5. Let me (Gautham) know once you think you're done.

# Additional Cool Stuff
## Bag Files
Bag Files are recorded topic data that you can replay. For example, I've recorded camera data and joystick/odrive telemetry data and put it in the Google Drive; the container should automatically download the bag files when you start it. Check the bags directory, you should see 2 subfolders in there.

To run a bag file, go into the bags directory and run
```bash
ros2 bag play --loop <bag name>
```
You should see all the topics that bag recorded replaying their data. 

## Image Visualization
You can visualize image and pointcloud data using rviz2. 
Open a new terminal and run 
```bash
rviz2
```
Then open a web browser (like a regular one, whatever you use daily), and navigate to
```text 
localhost:6080
```

You should see something like this:
![VNC Demo](/md_images/image2.png)

Click on the big green button.
Then, you should see something like this: (if you ran rviz2)
![rviz2 Demo](/md_images/image3.png)

This is the visualization tool for ROS2. It allows you to view image or pointcloud or most other kinds of topics. I've mostly only used it for visualizing images and pointclouds though.
To view a specific topic, 
1. click "panels" on the top left
2. click "add new panel"
3. click on the add button on the middle left of the screen (the one circled here)
![add button](/md_images/image4.png)

4. click "by topic" on the window that pops up
5. choose whatever topic you want
6. click "ok"

You should be able to see a new window show up visualizing that topic. 

The main point of this environment is to let you develop locally with bags and rviz2, and once you're pretty sure you got it write to clone the repo on the club laptop and run it with the real camera/odrives. 