
all: emulate_kinova_api.so

API_HEADERS_DIR:=headers.50104
KINOVA_H=$(API_HEADERS_DIR)/Kinova.API.CommLayerUbuntu.h $(API_HEADERS_DIR)/Kinova.API.UsbCommandLayerUbuntu.h $(API_HEADERS_DIR)/KinovaTypes.h

emulate_kinova_api.so: emulate_kinova_api.cc $(KINOVA_H)
	$(CXX) -fPIC -g -shared -o $@ -I$(API_HEADERS_DIR) $< 

clean: 
	-rm emulate_kinova_api.so

.PHONY: all clean

