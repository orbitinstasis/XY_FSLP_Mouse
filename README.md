# XY_FSLP_Mouse

![alt text](https://i.imgur.com/hizbWog.jpg)

Use a single sensor to get full mouse controls through USB OTG or BLE.

Note to self:
I'm going to refer to cursor manipulation as cursor mode, and when you're dragging as drag mode.

The raw data is just pressure and position. I have algorithms to handle joystick like control (up down left right) during drag mode. 

There's a bit more to it in cursor mode; the applied pressure non-linearly changes the left or right movement depending on the sensors position relative to the device (non-linear meaning a light to hard touch changes the movement speed). If the sensor's on the right of the device like in the video, then the pressure makes the cursor move left since that's where you're pushing. Holding a pressure within a tolerance holds the cursor still, pushing again from the still state resets the left movement speed so it moves slow to fast again depending on how hard you're pushing.

To move right you release the sensor - there's a 'gravity' on the side of the phone that has the sensor attached. The gravity is exponential, so once you release the sensor, the cursor moves right slow to fast. This is so if you overshot and want to go right a bit it won't fly past what you want. You can see this gravity effect in the video at 0:08. 
Moving up and down is done exponentially again but from static boundaries - the position on the sensor is divided up into three segments, top to bottom there's 'moving up' segment, 'only left right', and 'moving down' segment. 

Holding the force still keeps the cursor still if you're not moving up or down, if the applied pressure is greater than this tolerance then the cursor moves to the left as mentioned before; if the applied pressure is less than the tolerance (you ease off from the hold state) then the cursor is stationary indefinitely until you start increasing the pressure. This is useful because of the following: if the applied pressure is within the tolerance for a certain amount of time then a state is entered that triggers a left click when you release the sensor. If you don't release to trigger a left click after a certain period, then the drag mode is enabled (which has up down left right; left and right movement are relative to a 'zeroed' pressure point, this point is the pressure that was applied when you entered the drag mode); left and right movement is exponential depending on how far your applied pressure is from this zero point. 

To make all this fluid i've added visual and haptic cues - in the form of LEDs and a vibrating motor (not the one in the phone). I have different combinations of LEDs and vibrating patterns / intensities depending on what state you're in / if you clicked.
Sensitivity/time to enter states etc can be calibrated.
