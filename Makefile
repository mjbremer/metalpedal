EXE := prototype

SRC_DIR := Src
OBJ_DIR := obj

SRC := $(wildcard $(SRC_DIR)/*.c)

//OBJ := $(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

OBJ := $(OBJ_DIR)/prototype.o $(OBJ_DIR)/kiss_fft.o


CFLAGS := -Wall -Wpedantic -g -std=c99 -DPITCH_DETECT_LIB_DEBUG
CPPFLAGS := -IInc
LDFLAGS := -Llib


LDLIBS := -lm


all: $(EXE)

$(EXE): $(OBJ)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@


$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir $@


clean:
	$(RM) -r $(OBJ_DIR)

.PHONY: all clean
