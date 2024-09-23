## [Task 1C](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1c.html#task-1c)

# [Image Processing for Creating a 2D Map 🗺️](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1c.html#image-processing-for-creating-a-2d-map-)

### Aim 🎯

The objective of Task 1C is to develop essential image processing skills that will contribute to creating a 2D map for a drone to navigate in the environment.

### [Prerequisites 📋](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1c.html#prerequisites-)

It is presumed that you have successfully gone through the [image processing resources](https://portal.e-yantra.org/themeBook/wd/Learnings/Image_Processing/image_processing.html) provided in  **Learning Section** . Also this task involves writing programs in Python/C++ programming language, so get acquainted with the basics of Python and C++ and OpenCV libraries.

### [Installations 📥](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1c.html#installations-)

Before proceeding further, you need to install the some packages. To do that, follow these commands:

* Install **openCV** - This package provides various tools and algorithms for image and video processing, object detection, feature extraction, and more.
  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">pip3 install opencv-contrib-python
  </code></pre>
* Install **imutils** - This package make basic image processing functions such as translation, rotation, resizing, skeletonization, displaying Matplotlib images, sorting contours, detecting edges, and much more easier with OpenCV and Python.
  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">pip3 install imutils
  </code></pre>
* Install **NumPy** - This package is used for working with arrays. It has functions for working in domain of linear algebra, fourier transform, and matrices.
  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">pip3 install numpy
  </code></pre>

### [Task 📝](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1c.html#task-)

* Apply **Perspective Transform**
  ![](https://portal.e-yantra.org/themeBook/wd/Task_1/images/perspective_transform.jpg)

  * Load the given image `task1c_image.jpg` into your image processing environment.
  * Use an Aruco detection algorithm to locate and identify the four Aruco markers in the image. These corners will serve as the reference points for the perspective transform.
  * Define a set of four destination points that correspond to the desired output perspective. These points should form a rectangular or square shape in the transformed image. Ensure that the order of the destination points matches the order of the Aruco marker points.
  * Use the identified Aruco marker points and the defined destination points to calculate the perspective transformation matrix. Apply the perspective transformation to the original image, producing a new image with the desired perspective.
* Find **Obstacles**

  * Use transformed image, and then find the obstacles in the given image using openCV algorithms (understand the goal of this task and refer to [learning resources](https://portal.e-yantra.org/themeBook/wd/Task_1/(../Learnings/Image_Processing/image_processing.html)) to find out right algorithm for this problem).
* At the end you need to generate a `.txt` file which should contain

  * List of Aruco's detected
  * No of Obstacles
  * Total Area covered by obstacles

### [Procedure 📑](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1c.html#procedure-)

* Navigate to your workspace.

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">cd pico_ws/src/swift_pico/scripts/
  </code></pre>
* Open Python file `map.py` or CPP file `map.cpp` as per your choice and complete the script to achieve the given task.
* Add an argument parser in the script to accept the path of image to be used for detection. The argument should be named as --image . Refer this resources to understand: [How to add arguments in python](https://docs.python.org/3/library/argparse.html) and [How to add arguments in CPP](https://www.geeksforgeeks.org/command-line-arguments-in-cpp/) should be as follows.

  * For Python
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">python3 map.py --image task1c_image.jpg
    </code></pre>
  * For CPP
    <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">g++ -o map map.cpp $(pkg-config --cflags --libs opencv4)
    ./map --image task1c_image.jpg
    </code></pre>

  > **NOTE :** DO NOT hard code the name of the image to be processed, the image will be passed using the argument parser as shown above, the evaluation will be done by passing different test images when you submit on the portal.
  >
* The script should generate an output image as shown in the example below.
  ![](https://portal.e-yantra.org/themeBook/wd/Task_1/images/input.jpg) Img1: Input Image

  ![](https://portal.e-yantra.org/themeBook/wd/Task_1/images/output.png) Img2: Output Image
* And also a `.txt` file as shown in the example below.
  ![](https://portal.e-yantra.org/themeBook/wd/Task_1/images/text.png)

### [Submission Instructions 💯](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1c.html#submission-instructions-)

* **Step 1:** Rename the `.py` or `.cpp` file to `WD_<team_id>_map.py` or `WD_<team_id>_map.cpp`
* **Step 2:** Create a `zip` file containing this Python script or CPP script.

  > **Note: **The zip should contain only 1 file in the root directory, **DO NOT** make a folder and then zip, you should directly zip the file. You can use this command to zip the file. Use this command only after you have renamed the files. replace with your team id, for eg. if your team id is 1234, then the file name should be `WD_1234_map.py` / `WD_1234_map.cpp`.
  >

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">zip -r WD_<team_id>map.py
  </code></pre>

  OR

  <pre><div class="buttons"><button class="fa fa-copy clip-button" title="Copy to clipboard" aria-label="Copy to clipboard"><i class="tooltiptext"></i></button></div><code class="hljs">zip -r WD_<team_id>map.cpp
  </code></pre>

  This should be the structure of zip file
  |__WD_1234.zip
  … |__WD_1234_map.py/cpp
* **Step 3:** Submit this zip file on the portal and check your score ✨.

> **Points to be remebered before submitting Task1C:
> **
>
> * For this task, use only these packages **imutils, scikit-image, pandas, matplotlib, numpy, opencv-python, kneed, scikit-learn, common **for python and only **openCV** for C++. If you will use any other package grader won't evaluate your task.
> * Also make sure you follow the right format while generating the text file as shown above.
> * Your code should also take the argument **--image** for image input.DO NOT hard code the name of the image to be processed.
> * DO NOT use **imshow** while submitting your code on portal.

### [Deadline ⏳](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1c.html#deadline-)

Deadline for Task 1C is 16th October 23:59 hrs---

### [ALL THE BEST!! 🌟](https://portal.e-yantra.org/themeBook/wd/Task_1/task_1c.html#all-the-best-)
