### Crazyflie Indoor Position Logger Controller
is a tool developed for two main reasons:
- to get estimated position from the crazyflie
- to control the quadcopter so that it follow a generated path

if you like this project leave a start. Thanks!

The software is composed by three different part:
- a simple GUI for the user
- a logger
- a controller (in this case an offline LQR controller)

## The GUI
The user interface is mainly composed by three pages:
- the first one used to generate the desired path
![Image](images/image1.png)

- the second one to connect with a PS3 JoyStick
![Image](images/image2.png)

- the third one to configure the main setup of the logger and the controller.
![Image](images/image3.png)

The configuration is done by two different json files `config.json` and `sequence.json`. The first one, `config.json`, has the following structure:
```
{
	"crazyflies": [
	  {
	    "name": "Object-0",
	    "link": "radio://0/120/2M"
	  }
	],
	"nodes": [
		[0.00, 0.00, 2.17],
		[2.10, 0.60, 0.64],
		[2.77, 2.75, 1.85],
		[1.18, 3.01, 0.77]
	],
	"autoconnect": 1,
	"show_realtime_plots": 0
}
```
Where:
- **crazyflies:** contains a list of pair ({name=optional}, {link=the uri of the crazyflie}) 
- **nodes:** contains all the position of the nodes into the room
- **autoconnect:** is a boolean value (if 1 the app will connect to the crazyflie automatically)
- **show_realtime_plots:** is a boolean value (if 1 two plot related to the position and the attitude will be shown)

The second one (`sequence.json`), instead simply contains a list of point in 4D ([x, y, z]-coords + time in seconds). So for example the list can be:
```
[
  [0.640, 1.11, 1.7, 1],
  [0.704, 1.11, 1.7, 1],
  [0.768, 1.11, 1.7, 1]
]
```
this list can be simply generated throught the first page of the GUI.

## Logger + Controller
This is a the main interface of the logger and the controller:
![Image](images/image4.png)

In particular:
- the red sphere are the uwb-nodes
- the green sphere is the current setpoint
- the cube is the crazyflie

The main file that implements our virtual quadcopter is `drone_quaternion.py`. It is composed by three different parts:
- LQR controller
- NonLinear Observer of the quadcopter model
- Quadcopter model (useful for prediction but not used in this moment)

## Mathematical Model
The state of the drone is the following: [q0, q1, q2, q3, wx, wy, wz, px, py, pz, vx, vy, vz]'.
Where:
- [q0, q1, q2, q3] are the quaternions
- [wx, wy, wz] are the angular velocities
- [px, py, pz] are the linear position
- [vx, vy, vz] are the linear velocity

In terms of mathematical equation:

	Q=[
	-q(2) -q(3) -q(4);
	 q(1) -q(4)  q(3);
	 q(4)  q(1) -q(2);
	-q(3)  q(2)  q(1)
	];

	R = [
	 q(1)*q(1)+q(2)*q(2)-q(3)*q(3)-q(4)*q(4), 2*q(2)*q(3)-2*q(1)*q(4), 2*q(2)*q(4)+2*q(1)*q(3);
	 2*q(2)*q(3)+2*q(1)*q(4), q(1)*q(1)-q(2)*q(2)+q(3)*q(3)-q(4)*q(4), 2*q(3)*q(4)-2*q(1)*q(2);
	 2*q(2)*q(4)-2*q(1)*q(3), 2*q(3)*q(4) + 2*q(1)*q(2), q(1)*q(1)-q(2)*q(2)-q(3)*q(3)+q(4)*q(4);
	];

	M = [
	 0 -omega(3) omega(2); 
	 omega(3) 0 -omega(1); 
	 -omega(2) omega(1) 0
	 ];

	J = [
	 m*d^2 0 0;
	 0 m*d^2 0;
	 0 0 2*m*d^2
	];

	F_b = [0 0 F_tot]';
	q_dot = 0.5*Q*omega; 
	w_dot = drone.Mat_Jinv*(u(2:4)-M*J*omega);
	p_dot = v;
	v_dot = (1/m)*(R*F_b) - [0 0 g]';

Where the costants have the following values:

	g = 9.81;  %gravity acceleration in [m/s^2]
	m = 0.027; %drone mass in [Kg]
	d = (65.0538/1000)*sin(pi/4); %distance between motor's center and quadcopter's center in [m]

## Control
After the linearization around a stability point we can calculate the LQR gain matrix (`K`). However inside the model the input vector is composed by 4 components:

	[
	F1+F2+F3+F4; //total thrust
	Mx;          //moment around x-axis
	My;          //moment around y-axis
	Mz;          //moment around z-axis
	]

while the crazyflie accepts only the tuple (roll, pitch, yaw, thrust) for setpoints. So we need to convert these input in that way:

	scale = 65536.0 / (f_max * 4)
	u[0, 0] = u[0, 0]*scaleFactor
	u[1, 0] = (u[1, 0]/2.0)/self.d
	u[2, 0] = (u[2, 0]/2.0)/self.d
	u[3, 0] = u[3, 0]/self.c

	#m1 = u[0, 0] - u[1, 0] + u[2, 0] + u[3, 0]
	#m2 = u[0, 0] - u[1, 0] - u[2, 0] - u[3, 0]
	#m3 = u[0, 0] + u[1, 0] - u[2, 0] + u[3, 0]
	#m4 = u[0, 0] + u[1, 0] + u[2, 0] - u[3, 0]

so we can get the right (roll, pitch, thrust) in that way:

	t = (m1 + m2 + m3 + m4)/4 = u[0, 0]
	r = (m4 + m3 - m2 - m1)   = 4*u[1, 0]
	p = (m1 + m4 - m2 - m3)   = 4*u[2, 0]

after we can send this command to the crazyflie with the `cflib`

	self._cf.commander.send_setpoint(r, p, 0, int(t))

## NonLinear Observer
The NonLinear Observer was developed according: [Link](http://ing.univaq.it/manes/FilesLavoriPDF/R002_Observer_NonlinAn-TMeA_97.pdf)

## Installation

Simply clone/download the repo and execute on a terminal:

	git clone https://github.com/capriele/Crazyflie-Indoor-Position-Logger-Controller.git
	cd Crazyflie-Indoor-Position-Logger-Controller
	python app.py

Probably you need to install some library to run the project. This is the main list of the software i have installed:
- panda3D
- htmlPy + PyQt
- PyGame
- scipy + control + slycot python packages

## Support or Contact

If you have same trouble with this app please contact me at: [petrucci.alberto@gmail.com](mailto:petrucci.alberto@gmail.com).

