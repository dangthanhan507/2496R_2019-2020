This is Code for V1:

Put any documentation or notes under here.

robotmesh API website:
https://www.robotmesh.com/docs/vexv5-cpp/html/index.html

How to Compile Code with rmbuild:
    
    -First you must download the CODE on your computer.
        -On the AzureDevops repo that you are probably reading this README from, right click on the 2496R 2019-2020 UNDER the
            dropdown box labeled master.(There should also be a name called V1 to make sure you are right clicking the right one)
        -After right clicking, click Download as ZIP and then extract the files onto your Desktop

    -Run Powershell in your Windows 10 Computer
        -Once you run it, you will be typing on the line that displays
            ->  PS C:\Users\(Your PC Name)> _
        -Type cd '.\Desktop\2496R Code 2019-2020\V1'
    
    -Run rmbuild after navigating
        -Type 'rmbuild v5 (insert all .cpp files here) and run the code.
        Note: rmbuild compiles and runs the code at the same time, so there will be no object file, so that means you use this to 
        upload code too.

How to Compile Code with make.bat file:

    -Just click on make.bat

Info on files:

    main.cpp 
        -this file is where all the code meets into one place. Every code will eventually go into this one file for compilation
        which is also why it is the code that we compile.
        -this file requires four functions: drivercontrol(), pre_auton(), autonomous(), and main()
            -pre_auton() is a function that you run in the main() which will be the thing that runs even when robot is disabled
                -the reason it can run when the robot is disabled is because, 'disabled' means that it doesn't run the 
                drivercontrol() or autonomous() as a thread, so the only thread running will be main, and the only function 
                running in main is pre_auton()
                -since pre_auton can run in disabled, it is best to have lcd_selection or sensor setups inside pre_auton().

            -autonomous() is a thread that runs in main() that will only run when turned on via comp switch, field switch, 
            or the built-in switch on controller.
                -autonomous() is going to use the red_autons.cpp() and the blue_autons.cpp() to get the code to run on it.
                    -generally, this code will be short because we want to put all the details of a complex autonomous code into 
                    other files like blue_autons.cpp, red_autons.cpp, and auton_lib.h (auton_lib.cpp)

            -drivercontrol() is a thread that runs in main() that will only run when the robot is switched on and is not disabled
                -the code making this up will mostly be in main.cpp because it is not that long and it is much easier to access 
                this way.
                -drive code is divided up into drive_claw(), drive_chassis(), and drive_lift() to make sure those three different 
                codes do not intertwine. 

            main()
                -this is the function that automatically runs in every .cpp file code if put in there.
                -However, there will not be a lot of code put into main because VEX splits drivercontrol and autonomous as separate
                threads away from main, making main only a code u put in there to declare drivercontrol and autonomous as threads 
                via robotmesh competition API.

    config.h
        -This file serves to declare all the constants inside of the code for autonomous and drivercontrol.
        -This file will also declare all sensors for use.
        -This file contains the vex.h file used to access robotmesh API and PID.h to use PID on drivercontrol and autonomous
        -'using namespace vex;' removes you having to type 'vex::(robotmesh function)' because we are not about that life.
        -It is a header file to be declared on multiple files (does not need a .cpp file because there are not enough functions 
        written out fully to constitute it)

    auton_lib.h
        -This file serves as a middle man between auton_lib.cpp and red_autons.cpp and blue_autons.cpp
        -the reason why we need a middle man or a (header/linker file) is because auton_lib.cpp needs to be called twice 
        (one for red and blue)
        -you cannot call the same cpp file twice without getting a big error, so you will need a .h file to handle that for you 
        with its #ifndef and #define
        -extern serves the same function as #ifndef and #define but instead of handling files, this handles functions and 
        variables.

    auton_lib.cpp
        -This file contains all the library code for autonomous that will be used in red_autons.cpp and blue_autons.cpp
        -Very complex algorithms(curves) will be abstracted elsewhere, but generally, most auton commands
        (going forward, turning, resetting) goes here.

    blue_autons.cpp
        -This code contains functions that individually run an entire 15 second autonomous on blue side. 
        (so your whole list of autons should contain these functions)
    red_autons.cpp
        -This code contains functions that individually run an entire 15 second autonomous on blue side. 
        (so your whole list of autons should contain these functions)
        NOTE: the reason why blue_autons.cpp and red_autons.cpp are not combined to be the same thing is to separate red 
        from blue and make it easier to go through. 

    PID.h
        -This code contains all code for PID abstracted
        -it is written as a class in case there are other custom PID methods that would be written, but for now there is only 
        Calculate.
        -if you are going to write another method, update this README
        -(explanation of this algorithm on another file)
        -Note: to use this class, you are going to have to know how to use, write, and declare classes in c++ (ask jonathan or an 
        or stackoverflow)

    MotionProfile.h
        -This code abstracts Motion Profiling and the kinematics of the motors of the chassis.
        
        -in the one-dimensional application (going in a straight line and turning), this code will decrease error, and increase 
        consistency in the code. It will also control the acceleration of the motors which will increase its life-span.
        
        -in the two-dimensional application, it will make sure your code will follow the complex path you give it, and keep your 
        robot from straying from its path. 
        -(an s-curve motion profile is better, but depending on the time, trapezoidal is fast in terms of tuning)


            
