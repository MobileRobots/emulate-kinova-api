
ifndef SWIG
SWIG:=swig
endif

ifndef PYTHON_INCLUDE
PYTHON_INCLUDE:=/usr/include/python2.7
endif

DEFAULT_API_HEADERS_VER:=52000
DEFAULT_HEADERS_DIR:=headers.$(DEFAULT_API_HEADERS_VER)

CXXFLAGS=-fPIC -g -std=c++11

all: emulate_kinova_50101.so emulate_kinova_50104.so libemulate_kinova_50104.so libemulate_kinova_50101.so emulate_kinova_50200.so libemulate_kinova_50200.so emulate_kinova_52000.so libemulate_kinova_52000.so

lib%.so: %.so
	ln -s $< $@

KINOVA_HEADER_FILES:=$(DEFAULT_HEADERS_DIR)/Kinova.DEFAULT.CommLayerUbuntu.h $(DEFAULT_HEADERS_DIR)/Kinova.DEFAULT.UsbCommandLayerUbuntu.h $(DEFAULT_HEADERS_DIR)/KinovaTypes.h

info:
	$(info DEFAULT_API_HEADERS_VER=$(DEFAULT_API_HEADERS_VER) )
	$(info DEFAULT_HEADERS_DIR=$(DEFAULT_HEADERS_DIR) )
	$(info KINOVA_HEADER_FILES=$(KINOVA_HEADER_FILES) )
	$(info SWIG=$(SWIG) )
	$(info PYTHON_INCLUDE=$(PYTHON_INCLUDE) )
  

emulate_kinova_50104.so: emulate_kinova_api.cc
	$(CXX) $(CXXFLAGS) -shared -o $@ -DAPI_HEADERS_VER=50104 -Iheaders.50104 $<

emulate_kinova_50101.so: emulate_kinova_api.cc
	$(CXX) $(CXXFLAGS) -shared -o $@ -DAPI_HEADERS_VER=50101 -Iheaders.50101 $<

emulate_kinova_50200.so: emulate_kinova_api.cc
	$(CXX) $(CXXFLAGS) -shared -o $@ -DAPI_HEADERS_VER=50200 -Iheaders.50200 $<

emulate_kinova_52000.so: emulate_kinova_api.cc
	$(CXX) $(CXXFLAGS) -shared -o $@ -DAPI_HEADERS_VER=52000 -Iheaders.52000 $<


#50101
#DEFAULT_API_HEADERS_DIR=$(DEFAULT_HEADERS_DIR)
#emulate_kinova_api.so: emulate_kinova_api.cc 
#	$(CXX) $(CXXFLAGS) -shared -o $@ -DAPI_HEADERS_VER=$(DEFAULT_API_HEADERS_VER) -I$(DEFAULT_API_HEADERS_DIR) $< 

clean: 
	-rm emulate_kinova_*.so libemulate_kinova_*.so


_kinovapy.so: kinovapy.cc emulate_kinova_$(DEFAULT_API_HEADERS_VER).so
	$(CXX) $(CXXFLAGS) -DAPI_HEADERS_VER=$(DEFAULT_API_HEADERS_VER) -shared -o $@ -I$(DEFAULT_HEADERS_DIR) -I$(PYTHON_INCLUDE) $< -L.  -lemulate_kinova_$(DEFAULT_API_HEADERS_VER)

kinovapy.cc kinovapy.py: swig_wrapper.i $(DEFAULT_HEADERS_DIR)/Kinova.API.CommLayerUbuntu.h $(DEFAULT_HEADERS_DIR)/Kinova.API.UsbCommandLayerUbuntu.h $(DEFAULT_HEADERS_DIR)/KinovaTypes.h
	$(SWIG) -Wall -c++ -python -modern -module kinovapy -I$(DEFAULT_HEADERS_DIR) -o kinovapy.cc swig_wrapper.i

python: _kinovapy.so kinovapy.py

.PHONY: info clean python

