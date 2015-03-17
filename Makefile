

all: emulate_kinova_50101.so emulate_kinova_50104.so


#KINOVA_H=$(API_HEADERS_DIR)/Kinova.API.CommLayerUbuntu.h $(API_HEADERS_DIR)/Kinova.API.UsbCommandLayerUbuntu.h $(API_HEADERS_DIR)/KinovaTypes.h

emulate_kinova_50104.so: emulate_kinova_api.cc
	$(CXX) -fPIC -g -shared -o $@ -DAPI_HEADERS_VER=50104 -Iheaders.50104 $<

emulate_kinova_50101.so: emulate_kinova_api.cc
	$(CXX) -fPIC -g -shared -o $@ -DAPI_HEADERS_VER=50101 -Iheaders.50101 $<


#DEFAULT_API_HEADERS_VER=50104  
#50101
#DEFAULT_API_HEADERS_DIR=headers.$(DEFAULT_API_HEADERS_VER)
#emulate_kinova_api.so: emulate_kinova_api.cc 
#	$(CXX) -fPIC -g -shared -o $@ -DAPI_HEADERS_VER=$(DEFAULT_API_HEADERS_VER) -I$(DEFAULT_API_HEADERS_DIR) $< 

clean: 
	-rm emulate_kinova_*.so


