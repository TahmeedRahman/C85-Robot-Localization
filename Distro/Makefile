CXX = g++
CXXFLAGS = -Wall -Wextra
BT = ./EV3_RobotControl/btcomm.c
LIBS = -lbluetooth

all: localization colour_calibration

localization: EV3_Localization.c $(BT)
	$(CXX) $(CXXFLAGS) EV3_Localization.c $(BT) $(LIBS) -o localization

colour_calibration: colour_calibration.c $(BT)
	$(CXX) $(CXXFLAGS) colour_calibration.c $(BT) $(LIBS) -o calibrate_colours

clean:
	rm -f localization colour_calibration