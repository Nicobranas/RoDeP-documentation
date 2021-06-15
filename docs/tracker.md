#Purpose of the line tracker

<p align = justify>The line tracker allows the robot to move autonomously in its storages areas. The robot is able to follow a line and recognize Apriltags to stop when it is necessary.</p>

##The RODEP's work area

<p align = justify>Here is an example of the orgazination of the robot's work area :<br/></p>


##Line tracker operation

<p align = justify>The Raspberry Pi Camera allows to obtain RGB images. These images are passed to HSV format in order to permform a thresholding on them thanks to OpenCV.<br/>

The principle of thresholding is to set a minimum and a maximum for the value of H, S and V. For each pixel in HSV image, the function test if the value of H, S and V are between the minimum and the minimum. If it is the case then the pixel turns to white otherwise it turns to black.<br/>

Here is 3 lines of the Python code to obtain the 3 mask with the thresholding :<br/></p>

<code> apriltag\_mask = 255-cv2.inRange(frame\_HSV, (140,110,0), (180,200,255)) 
blue\_line\_mask = 255-cv2.inRange(frame\_HSV, (110,170,0), (130,255,255)) 
green\_line\_mask = 255-cv2.inRange(frame\_HSV, (80,60,0), (110,170,255)) </code>

<p align = justify>For the next step it is neccessary to have the objects in black on a white background. So, this is the reason of the <code>255-cv2.inrange(...)</code> to invert the colors.<br/>

Here are examples of masks obtained with the thresholding :<br/></p>

##Error calculation

<p align = justify>Now that the lines are detected, the error must be defined and calculated to allow the robot to modifify its trajectory. The chosen line tracking technique is the one where we take the average of the pixels belonging to the line and we want to realign this average with the center of the image. So, the calculated error is the difference between the average of the pixels belonging to the line and the center of the image.<br/>

The following scheme represent and define the error :<br/>

<p align = justify>This error is used for the motor control. The operation of the motor control is explained on the next page.<br/></p>

