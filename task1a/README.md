## [Task 1A](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#task-1a)

# [Basics of ROS 2 using Turtlesim 🐇](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#basics-of-ros-2-using-turtlesim-)

### Aim 🎯

The objective of Task 1A is to create a simple controller for **turtlesim** using Python/C++ and use it to perform the desired maneuver with turtle (or drone???) exactly as described below.

### [Prerequisites 📋](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#prerequisites-)

It is presumed that you have successfully gone through the [**turtlesim resources**](https://portal.e-yantra.org/themeBook/wd/Learnings/ROS/turtlesim_tutorials.html) provided in  **Learning Section** . Also this task involves writing programs in Python/C++ programming language, so get acquainted with the basics of Python/C++ .

### [Task 📝](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#task-)

* Task is to create a 2D drone drawing using turtle/s as shown below

![ros2 turtle controller output](https://portal.e-yantra.org/themeBook/wd/Task_1/images/task1A_Final.png)

* There are two things that need's to be done:

  * First, draw **four circles** with centers at  **(2.0,2.0)** ,  **(2.0,8.0)** ,  **(8.0,8.0)** , and  **(8.0,2.0)** , each having a diameter of  **2.0** , using the turtle to represent the propellers.
  * Next, draw the drone frame, which is a **square** with its center at  **(5.0,5.0)** . The vertices of the square are at  **(3.0,5.0)** ,  **(5.0,7.0)** , **(7.0,5.0)** and  **(5.0,3.0)** .
  * Then draw the **lines** to complete the frame as shown in the picture.
  * At the end, turtle/drone should come at the center of square i.e  **(5.0, 5.0)** .

  The final output/drawing should resemble the appearance of a drone as shown in the picture above.

### [Installations 📥](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#installations-)

* Create a ROS2 Workspace
  Since we will be working with custom packages and customised nodes, we will be creating a workspace where all our changes get incorporated at once when we source the specific workspace. We recommend to create a directory named `turtlesim_ws` for this task.

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd
  mkdir -p turtlesim_ws/src 
  </code></pre>

  ( *in `-p` mode, `mkdir` command creates the parent directories if not already created* )
* Clone the repository from Github

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd ~/turtlesim_ws/src 
  git clone -b turtle https://github.com/eYantra-Robotics-Competition/eyrc-24-25-warehouse-drone.git 
  </code></pre>
* Navigate to the workspace folder

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd ..
  </code></pre>
* Build the workspace.

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">colcon build 
  </code></pre>
* Source your workspace by following command

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">source install/setup.bash
  </code></pre>

### [Procedure 📑](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#procedure-)

> **Note:** You can write the node script using either Python or C++ programming languages.

* Creating a new package inside the ROS2 Workspace

  #### [For Python developers](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#for-python-developers)


  * To create a python package navigate to the `src` folder using `cd src` inside the workspace and run the following command:

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="language-bash hljs">ros2 pkg create --build-type ament_python wd_task_1a
    </code></pre>

    Now you have successfully created a new package. As you learnt in the [ROS 2 Learning Resources](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), in this package we will create the **task_1a** node that will be called to perform the desired task!
  * Create a Python script for task1A,

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="language-bash hljs">touch task_1a_<team_id>.py
    </code></pre>

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="language-bash hljs">chmod +x task_1a_<team_id>.py
    </code></pre>

    where `<team_id>` is the is your team ID. For example, if your team ID is 1234, you should create the file names `task_1a_1234.py`.

    The `touch` command creates a python file and the `chmod` command converts it into an executable.
  * Navigate to the workspace folder

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd ~/turtlesim_ws
    </code></pre>
  * Build the workspace

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">colcon build 
    </code></pre>

  #### [For C++ developers](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#for-c-developers)

  * To create a C++ package navigate to the `src` folder using `cd src` inside the workspace and run the following command:

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 pkg create --build-type ament_cmake wd_task_1a
    </code></pre>

    Now you have successfully created a new package. As you learnt in the [ROS 2 Learning Resources](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), in this package we will create the **task_1a** node that will be called to perform the desired task!
  * Create a C++ script for task1A,

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">touch task_1a_<team_id>.cpp
    </code></pre>

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">chmod +x task_1a_<team_id>.cpp
    </code></pre>

    where `<team_id>` is the is your team ID. For example, if your team ID is 1234, you should create the file names `task_1a_1234.cpp`.

    The `touch` command creates a file and the `chmod` command converts it into an executable.
  * Navigate to the workspace folder

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd turtlesim_ws
    </code></pre>
  * Build the workspace.

    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">colcon build 
    </code></pre>
* Editing the Python/C++ file

  * Open the `task_1a_1234.py`/`task_1a_1234.cpp` file in your favourite code editor. (We highly recommend [VS Code](https://code.visualstudio.com/docs/setup/linux)!)
  * Now edit the `task_1a_1234.py`/`task_1a_1234.cpp` file and implement your logic. We have provided an example in learning resources to get you thoroughly acquainted.

Refer to [ROS2 Publisher-Subscriber and Service Example (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-python) / [ROS2 Publisher-Subscriber and Service Example (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

**Do not forget to add the dependencies in the `package.xml` file and add the new nodes in the `setup.py`(for Python package) and `CMakeLists.txt`(for C++ package)**

### [Submission Instructions 💯](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#submission-instructions-)

* **Step 1:**
  * To start the turtlesim, navigate to the workspace folder
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd ~/turtlesim_ws
    </code></pre>
  * Source your workspace
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">source install/setup.bash
    </code></pre>
  * Run the turtlesim node
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 run turtlesim turtlesim_node
    </code></pre>
* **Step 2:**
  * Start the bag recording using,
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 bag record -o task_1a --all 
    </code></pre>
* **Step 3:**
  * Run the Python script
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">ros2 run wd_task_1a task_1a_<team_id>
    </code></pre>
* **Step 4:**
  * Once your run is complete, stop the bag recording using `Ctrl + C`.
  * Take a screenshot of the entire screen.
  * Now zip all these 4 files and name the zip file as `WD_<team_id>_task_1a.zip`.Make sure you follow this structure of your zip file.
    This should be the structure of zip file
    |__WD_1234_task_1a.zip
    … |__task_1a_0.db3
    … |__metadata.yaml
    … |__WD_1234_task_1a.py/cpp
    … |__WD_1234_task_1a.png
* **Step 5:** Submit a zip file on the portal in the place of Task 1A and check your score.

### [Deadline ⏳](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#deadline-)

Deadline for Task 1A is 16th October 23:59 hrs

---

### [Turtle all the Way!! 🐢](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1a.html#turtle-all-the-way-)
