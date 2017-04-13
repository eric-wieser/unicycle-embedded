Terminal
========

To open the terminal to connect to the robot, run::

    cd tools
    python3 terminal.py

You'll be greeted with a ``yauc>`` prompt, into which you can type ``help`` for more information.

A typical session looks as follows:

..  code::

	...\tools> .\terminal.py
	Yaw Actuated UniCycle command line interface
	Connected!
	Yaw Actuated UniCycle startup
	Starting PWM setup
	Starting I2C setup
	Starting encoder setup
	All done
	yauc> policy ../ctrl.mat
	controller {
	  wheel {
	    k_pitch: 1.2732395447351628
	  }
	  turntable {
	    k_dyaw: -0.6366197723675814
	  }
	}

	yauc> go 5
	Starting bulk mode
	Asking for logs
	Waiting for reply
	No data yet
	Asking for logs
	Waiting for reply
	No data yet
	Asking for logs
	Waiting for reply
	Test completed
	Sending test data
	Success
	Saved rollout of 5 steps to ..\logs\2017-04-04 10꞉25꞉50\0.mat
	yauc> disconnect
	Connection lost
	yauc> ^D
	Exiting cli and stopping motors
	Already disconnected
	yauc>

After using the  ``go`` command, the USB cable can be disconnected. The terminal will automatically reconnect to and recieve data from the robot when the USB cable is reattached.

.. autoclass:: terminal.Commands
   :members:

