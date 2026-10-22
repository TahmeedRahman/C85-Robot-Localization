CXX = g++
# CXXFLAGS = -Wall -Wextra
BT = ./EV3_RobotControl/btcomm.c
LIBS = -lbluetooth

all: localization calibrate_colours

localization: EV3_Localization.c $(BT)
	$(CXX) EV3_Localization.c $(BT) $(LIBS) -o localization.out

calibrate_colours: calibrate_colours.c $(BT)
	$(CXX) calibrate_colours.c $(BT) $(LIBS) -o calibrate_colours.out

clean:
	rm -f localization calibrate_colours