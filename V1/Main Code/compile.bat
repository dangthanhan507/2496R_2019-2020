powershell -Command "rmbuild v5 smallRed.cpp ../auton_src/auton_lib.cpp ../config.cpp  --slot 1"
powershell -Command "rmbuild v5 bigRed.cpp ../auton_src/auton_lib.cpp ../config.cpp  --slot 2"
powershell -Command "rmbuild v5 smallBlue.cpp ../auton_src/auton_lib.cpp ../config.cpp  --slot 3"
powershell -Command "rmbuild v5 bigBlue.cpp ../auton_src/auton_lib.cpp ../config.cpp --slot 4"
powershell -Command "rmbuild v5 skills.cpp ../auton_src/auton_lib.cpp ../config.cpp --slot 6"
cmd /k
