# HRI-pose_estimation

With this project I developed a robot for human-robot interaction: first I built a robot structure, then I implemented a software to help a human to execute some poses using the task of human-pose estimation. All the work is explained in the file 'report.pdf'

This repository contains:

- The foleder 'ROS packages' which contains 6 subfolders, where each folder correspond to a package of the ROS workspace created for the project and contains the implemented python files (except for the folder "customized messages and services", which contain 2 files of type '.msg' and  6 files of type '.srv').

- The folder 'PDDL' contains two subfolders, one contain the simplified domain studied, the other the complete detailed domain.

- The pdf file 'reasoning_agents_tools_used.pdf' contains all the implemnted tools of the reasoning agents part.

- 'server_pose_estimation.py' implementing a server for pose estimation: it receive an image and re-send it with pose estimation performed and corresponding keypoints.

- 'launch_application.py' which lauch all the ros nodes implemented for the execution of the task. Morevore it allow to the nodes to restart when the task is completed.

-  The file 'report.pdf' contains all the resume on the work done.
  
- At the link [https://youtu.be/h43wUQIjOj4?si=9pjG6-3qI83sy1zq] you can see a demo of operation.
