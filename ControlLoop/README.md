README - ControlLoop Testing

Use this to compile. You can only compile from the MonopropUAV/ControlLoop directory.
Make surethat you have g++ installed on your system.

g++ -std=c++17 -o data_mocker data_mocker.cpp loop.cpp init.cpp updatedMadgwick.cpp  \\
ExtendedKalmanFilterGeneral.cpp EKF_xy.cpp EKF_z.cpp lqr.cpp Vector.cpp \\
Matrix.cpp LQR/calculateA.cpp LQR/calculateB.cpp

To generate the plots, navigate to WSL from command prompt or get WSL on your system. 
It should look like this:

C:\Users\you\> wsl

you@YourLaptop:~$

Once you are there, create a directory:

you@YourLaptop:~$ mkdir gtpl
you@YourLaptop:~$ cd gtpl

Then pull the repository into gtpl:

you@YourLaptop:~/gtpl$ git pull main
you@YourLaptop:~/gtpl cd MonopropUAV/ControlLoop/data_creators

Source python libraries from the venv:

you@YourLaptop:~/gtpl/MonopropUAV/ControlLoop/data_creators$ source dataEnv/bin/activate

Then run this Python program. It should automatically open a Plotly tab in Chrome.

(dataEnv) you@YourLaptop:~/gtpl/MonopropUAV/ControlLoop/data_creators$ python3 plot_state.py

--> Opens Plotly Tab in Chrome

If you can't do any of this for some reason, go to the file
(MonopropUAV/ControlLoop/data_creators/plot_state.py) and ask chatgpt
to modify the lines with "subprocess" in them to make it work for your
operating system.

