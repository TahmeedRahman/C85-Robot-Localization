CXX = g++
CXXFLAGS = -Wall -Wextra
BT = ./EV3_RobotControl/btcomm.c
LIBS = -lbluetooth

all: localization calibrate_colours

localization: EV3_Localization.c $(BT)
	$(CXX) $(CXXFLAGS) EV3_Localization.c $(BT) $(LIBS) -o localization

calibrate_colours: calibrate_colours.c $(BT)
	$(CXX) $(CXXFLAGS) calibrate_colours.c $(BT) $(LIBS) -o calibrate_colours

clean:
	rm -f localization calibrate_colours