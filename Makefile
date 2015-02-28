
all: emulate_kinova_api.so

emulate_kinova_api.so: emulate_kinova_api.cc Kinova.API.CommLayerUbuntu.h Kinova.API.UsbCommandLayerUbuntu.h KinovaTypes.h
	$(CXX) -fPIC -g -shared -o $@ $< -lm

clean: 
	-rm emulate_kinova_api.so

.PHONY: all clean

