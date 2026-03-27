# Webots Makefile for C++ Vehicle Controller

# 1. 소스 파일 이름 확인
CXX_SOURCES = autonomous_vehicle.cpp

# 2. C++ 전용 차량 라이브러리 추가 (CppDriver, CppCar)
# -lCppDriver: webots::Driver 클래스 구현체
# -lCppCar: webots::Car 클래스 구현체 (필요 시)
LIBRARIES = -lCppDriver -lCppCar -ldriver -lcar

# ----------------------------------------------------------------
# 3. Webots 시스템 기본 설정
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

include $(WEBOTS_HOME_PATH)/resources/Makefile.include
# ----------------------------------------------------------------