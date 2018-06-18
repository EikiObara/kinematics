  SRC_DIR    = ./src
ifeq "$(strip $(SRC_DIR))" ""
  SRC_DIR  = .
endif
INC_DIR    = ./include
ifeq "$(strip $(INC_DIR))" ""
  INC_DIR  = .
endif
OBJ_DIR    = ./obj
ifeq "$(strip $(OBJ_DIR))" ""
  OBJ_DIR  = .
endif
BIN_DIR    = ./bin
ifeq "$(strip $(BIN_DIR))" ""
  BIN_DIR  = .
endif

COMPILER  = g++
CXXFLAGS    = -MMD -MP -Wall -Wextra -O0 -std=c++11
LDFLAGS   = -lpthread -lEposCmd

TARGET    = $(BIN_DIR)/$(shell basename `readlink -f .`)
INCLUDE   = -I$(INC_DIR)
SOURCES   = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS   = $(addprefix $(OBJ_DIR)/, $(notdir $(SOURCES:.cpp=.o)))
DEPENDS   = $(OBJECTS:.o=.d)


$(TARGET): $(OBJECTS)
	$(COMPILER) -o $@ $^ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	-mkdir -p $(OBJ_DIR)
	$(COMPILER) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<


.PHONY: default
default: $(TARGET)

.PHONY: all
all: clean $(TARGET)

.PHONY: clean
clean:
	-rm -f $(OBJECTS) $(DEPENDS) $(TARGET)

TEST_DIR       = ./test
GTEST_DIR      = $(TEST_DIR)/extsrc/googletest/googletest
TEST_SRC_DIR   = $(TEST_DIR)/src
TEST_INC_DIR   = $(TEST_DIR)/include
TEST_BIN_DIR   = $(TEST_DIR)/bin
TEST_LIB_DIR   = $(TEST_DIR)/lib
TEST_OBJ_DIR   = $(TEST_DIR)/obj
TEST_SRCS      = $(wildcard $(TEST_SRC_DIR)/src/*.cpp)

#新しいテストを書いたらここに追加
TEST_TARGET    = sample1_unittest

TEST_OBJS      = $(addprefix $(TEST_OBJ_DIR)/, $(notdir $(TEST_SRCS:.cpp=.o)))
TEST_LIBS     += -L$(TEST_LIB_DIR)

CPPFLAGS      += -isystem $(GTEST_DIR)/include

#変更禁止
GTEST_HEADERS  = $(GTEST_DIR)/include/gtest/*.h \
                $(GTEST_DIR)/include/gtest/internal/*.h
#変更禁止
GTEST_SRCS_ = $(GTEST_DIR)/src/*.cc $(GTEST_DIR)/src/*.h $(GTEST_HEADERS)


gtest-all.o : $(GTEST_SRCS_)
	$(CXX) $(CPPFLAGS) -I$(GTEST_DIR) $(CXXFLAGS) -c \
            $(GTEST_DIR)/src/gtest-all.cc

gtest_main.o : $(GTEST_SRCS_)
	$(CXX) $(CPPFLAGS) -I$(GTEST_DIR) $(CXXFLAGS) -c \
            $(GTEST_DIR)/src/gtest_main.cc

gtest.a : gtest-all.o
	$(AR) $(ARFLAGS) $@ $^

gtest_main.a : gtest-all.o gtest_main.o
	$(AR) $(ARFLAGS) $@ $^


#新しいテストを書いたらルールを追加

#[テスト対象].o : $(SRC_DIR)/[テスト対象].cpp $(INC_DIR)/[テスト対象].h $(GTEST_HEADERS)
#	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE) -c $(SRC_DIR)/[テスト対象].cpp
#
#[テスト名].o : $(TEST_SRC_DIR)/[テスト名].cpp \
#                     $(INC_DIR)/[テスト対象].h $(GTEST_HEADERS)
#	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE) -c $(TEST_SRC_DIR)/[テスト名].cpp
#
#[テスト名] : [テスト対象].o [テスト名].o gtest_main.a
#	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -lpthread -pthread $^ -o $@


sample1.o : $(SRC_DIR)/sample1.cpp $(INC_DIR)/sample1.h $(GTEST_HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE) -c $(SRC_DIR)/sample1.cpp

sample1_unittest.o : $(TEST_SRC_DIR)/sample1_unittest.cpp \
                     $(INC_DIR)/sample1.h $(GTEST_HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDE) -c $(TEST_SRC_DIR)/sample1_unittest.cpp

sample1_unittest : sample1.o sample1_unittest.o gtest_main.a
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -lpthread -pthread $^ -o $@


.PHONY: test
test: $(TEST_TARGET)

.PHONY: test_clean
test_clean :
	rm -f $(TEST_TARGET) gtest.a gtest_main.a *.o *.d

-include $(DEPENDS)

