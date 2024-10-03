<h1>RC car STM32 Nucleo F411RE</h1>
RC car with the ability to be either controlled with Bluetooth module or with self-driving option.

<h2>Self-driving</h2>
Self-driving functionality is implemented by combining ultrasonic sensor with servo. Starting this mode (pushing the B1 button) causes this car to move, and then the measurements simultaneously  start to begin.
While scanning, in order to exclude any false results, median filter is implemented. When it approaches any obstacle, scanning of terrain begins; servo with attached sensor concludes whether turning left or right will provide more space for further movement.
<h2>Bluetooth</h2>
The RC car, can be controlled by sending steering commands through app enabling communication with serial interface. On the Android phone, the project was tested with app called "Serial Bluetooth Terminal", which allows binding commands for certain buttons, making it easier to control the vehicle.
<br>
<br>
<h2>Parts used</h2>
<ul>
<li>Main board used in this project is STM32 F411RE.</li>
<li>Two L298N motor drivers used for controlling wheels.</li>
<li>HC-SR04 for detecting distance.</li>
<li>Servo (in this example SG90) for moving attached on it sensor.</li>
<li>Bluetooth module (in this example Bluetooth 4.0 Low Energy Module).</li>
</ul>
