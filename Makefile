BOARD = tangnano9k
FAMILY = GW1N-9C
DEVICE = GW1NR-LV9QN88PC6/I5

OBJS = top.v
PROJECT = line_tracker

all: $(PROJECT).fs

$(PROJECT).json: $(OBJS)
	yosys -p "read_verilog $(OBJS); synth_gowin -top top -json $(PROJECT).json"

$(PROJECT)_pnr.json: $(PROJECT).json pins.cst
	nextpnr-himbaechel --json $(PROJECT).json --write $(PROJECT)_pnr.json --device $(DEVICE) --vopt family=$(FAMILY) --vopt cst=pins.cst

$(PROJECT).fs: $(PROJECT)_pnr.json
	gowin_pack -d $(FAMILY) -o $(PROJECT).fs $(PROJECT)_pnr.json

load: $(PROJECT).fs
	openFPGALoader -b $(BOARD) $(PROJECT).fs

flash: $(PROJECT).fs
	openFPGALoader -b $(BOARD) -f $(PROJECT).fs

clean:
	rm -f *.json *.pnr *.fs
