## [Task 1B](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#task-1b)

# [Stabilising the Swift Pico Drone](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#stabilising-the-swift-pico-drone)

### Aim 🎯

The aim of this task is to develop a PID control system to stabilize the Swift Pico Drone at any specified position within a Gazebo simulation environment.

### [Prerequisites 📋](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#prerequisites-)

It is assumed that you have successfully completed Task 0 and Task 1a, and have reviewed the [learning resources](https://portal.e-yantra.org/themeBook/wd/WD-learn-section-intro.html) for this task. You may write the source code for this task in either C++ or Python.

### [Installations 📥](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#installations-)

Before proceeding further, you need to install the following packages. To do that, follow these commands:

* Create a colcon workspace named `pico_ws`
  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd
  mkdir pico_ws
  </code></pre>
* Now clone the eyrc-24-25-warehouse-drone repository form GitHub
  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd ~/pico_ws
  git clone -b wd_sim https://github.com/eYantra-Robotics-Competition/eyrc-24-25-warehouse-drone.git --recursive src
  </code></pre>
* Now build your workspace
  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd ~/pico_ws
  colcon build
  </code></pre>
* Each time you build your workspace, you need to source setup.bash file from the `pico_ws/install` folder. Instead of doing this manually, let's add a line in `.bashrc`.
  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">echo "source ~/pico_ws/install/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  </code></pre>
* Install Plotjuggler to view graphs
  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">sudo apt install ros-humble-plotjuggler-ros
  </code></pre>
* Now you can go and visit [Understanding the Swift Pico model in Ignition Gazebo](https://portal.e-yantra.org/themeBook/wd/Learnings/Quadcopter/swift_model.html) to learn how to use Swift Pico drone.

### [Task 📝](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#task-)

* Develop a PID controller for the Swift Pico drone to control its position (x, y, z) within a Gazebo simulation.
* The PID controller will operate as a closed-loop system. The real-time position of the Swift Pico drone will be fed back into the controller as feedback.
* The controller will generate commands in the form of angle-setpoints, directing the Swift Pico drone to tilt accordingly.
* The PID controller should be implemented as a ROS 2 node, written in either Python or C++.
* Once the PID controller is built and successfully tuned, the Swift Pico drone should be able to move to and stabilize at the setpoint **[2, 2, 19]** in the Gazebo environment, maintaining an error margin of **±0.4** in all coordinates for atleast  **10s** .

### [Procedure 📑](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#procedure-)

* Navigate to your workspace.

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd pico_ws
  </code></pre>
* Set the `GZ_SIM_RESOURCE_PATH` environment variable to include the path to the `rotors_swift_description/models` directory.

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">export GZ_SIM_RESOURCE_PATH="$HOME/pico_ws/src/rotors_simulator/rotors_swift_description/models"
  </code></pre>
* Launch the Gazebo world containing the Swift Pico drone and the overhead camera by typing the following command :

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 launch swift_pico task_1b.launch.py
  </code></pre>
* For Python:

  * Thoroughly review the boilerplate Python program `pico_controller.py` located in the `~/pico_ws/src/swift_pico/src` folder. Complete the script by implementing the PID controller.
  * You can run the Python script using the following command to control the Swift Pico drone in new terminal:
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 run swift_pico pico_controller.py
    </code></pre>
* For C++:

  * Thoroughly review the boilerplate C++ program `pico_controller.cpp` located in the `~/pico_ws/src/swift_pico/src` folder. Complete the script by implementing the PID controller.
  * You can run the C++ script using the following command to control the Swift Pico drone in new terminal:
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 run swift_pico pico_controller_cpp
    </code></pre>
* To launch the PID Tuning GUI for tuning PID values, run the following command in new terminal:

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 launch pid_tune pid_tune_drone.launch.py node_name:=button_ui
  </code></pre>

  The video given below demonstrates how to use the PID Tuning GUI:

  <iframe width="728" height="409.5" src="https://www.youtube.com/embed/3fzePbqZaWo?si=SV80rC1qtUHl61D8&start=7" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen=""></iframe>
* Tune the PID controller to optimize the performance of the Swift Pico drone, ensuring it reaches and stabilizes at the desired setpoint for at least 10 seconds, ideally within 60 seconds. The faster the drone stabilizes, the better. Once tuning is complete, finalize the P, I, and D gains in your Python/C++ script.
* To launch Plotjuggler for visualizing ROS 2 topics, run the following command in new terminal:

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 run plotjuggler plotjuggler
  </code></pre>

  The video given below demonstrates how to use Plotjuggler to visualize ROS 2 Topics:

  <iframe width="728" height="409.5" src="https://www.youtube.com/embed/0XomsYGk0_Q?si=KO19aLdgo21DLpUS&start=4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen=""></iframe>
* Follow the recording and submission instructions to submit your task.

### [Recording and Submission Instructions 💯](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#recording-and-submission-instructions-)

* **Step 1:** Implement the PID controller and complete the Python/C++ script for hovering the drone at the desired point.
* **Step 2:** Tune the P, I and D values for roll, pitch and throttle using the PID Tuning GUI. After you are confident with your P, I and D gains, fix them in your code and make sure you multiply the constants that were used to scale down the gains in the call back function. For eg, if your P gain in GUI slider was 1000, and your multiplication factor in the callback function is 0.06, then the effective P gain is 60, so hardcode this gain in the script in the variables so that just by running the code, the PID controller starts. Do this for roll, pitch and throttle.
* **Step 3:** Now you need to record your submission, a tool named `ros2 bag` helps to record ROS 2 topics just as a video. When you feel confident with the performance of your PID controller and you are ready to record the submission, use another launch file which will run the same things as in `task_1b.launch.py` as well as start the `pico_controller.py`/`pico_controller.cpp` and a node to start the ros2 bag to record the WhyCon poses. Use this launch file to implement the task and record a bag file for 1 minute i.e. 60 second.

  * For Python:
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 launch swift_pico task_1b_submission_py.launch.py
    </code></pre>
  * For C++:
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 launch swift_pico task_1b_submission_cpp.launch.py
    </code></pre>
* **Step 4:** This will generate a folder named `task_1b` in the `pico_ws`, containing 2 files: `task_1b_0.db3` and `metadata.yaml`. Rename the Python script to `WD_<team_id>_pico_controller.py`. Now zip all these 3 files and name the zip file as `WD_<team_id>_task_1b.zip`.

  > **NOTE:** The zip should contain only 3 files in the root directory, DO NOT make a folder and then zip, directly zip the three files. You can use this command to zip the file. Use this command only after you have renamed the file. replace <team_id> with your team id, for eg. if your team id is 1234, then the file names should be `WD_1234_pico_controller.py` or `WD_1234_pico_controller.cpp`.
  >

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">zip -r WD_<team_id>_task_1b.zip WD_<team_id>_pico_controller.py task_1b_0.db3 metadata.yaml
  </code></pre>

  or

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">zip -r WD_<team_id>_task_1b.zip WD_<team_id>_pico_controller.cpp task_1b_0.db3 metadata.yaml
  </code></pre>

  This should be the structure of zip file
  |__WD_1234_task_1b.zip
  … |__task_1b_0.db3
  … |__metadata.yaml
  … |__WD_1234_pico_controller.py/cpp
* **Step 5:** Submit this zip file on the portal in the place of Task 1B and check your score.

#### [YouTube Video](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#youtube-video)

* Record the video using a screen recorder like kazam or simplescreen recorder. You can install them using the following commands:

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">sudo apt install kazam
  </code></pre>

  or

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">sudo apt install simplescreenrecorder
  </code></pre>
* Start recording the video, at the start terminal should be visble where you will run the launch file.
* Once you start your script your screen should have Gazebo, terminal and camera output as shown in the following image for most of the time:
  ![](https://portal.e-yantra.org/themeBook/wd/Task_1/images/YouTube_rec.png)
* Upload a one-shot continuous video with the title `<span>WD_<team_id>_Task_1b</span>` (For example: If your team ID is 1234 then, save it as WD_1234_Task_1b)
* Please note that while uploading the video on YouTube select the privacy setting option as `Unlisted`.
* Submit the YouTube link of your video on the portal.

### [Deadline ⏳](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#deadline-)

Deadline for Task 1B is 16th October 23:59 hrs.

---

### [🚁Happy Flying!!🕹](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1b.html#happy-flying)
