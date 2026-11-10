CXX      ?= c++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion \
            -fno-exceptions -fno-rtti
BUILD    := build

.PHONY: all test freestanding clean media

all: $(BUILD)/harness

$(BUILD)/harness: test/harness.cpp flight/*.hpp sim/*.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $@

# Compile the flight code with no hosted runtime at all: no libc, no libc++.
freestanding: | $(BUILD)
	$(CXX) $(CXXFLAGS) -ffreestanding -nostdlib -c flight/flight_core.cpp -o $(BUILD)/flight_core.o
	@echo "flight_core.o built freestanding; undefined symbols:"; nm -u $(BUILD)/flight_core.o || true

test: $(BUILD)/harness freestanding
	@mkdir -p telemetry
	./$(BUILD)/harness telemetry

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD) telemetry/*.csv

# LinkedIn/portfolio media: SVG blueprint (+ 2x PNG export) and 300 DPI telemetry plot.
CHROME ?= /Applications/Google Chrome.app/Contents/MacOS/Google Chrome
media: test
	python3 tools/gen_architecture_svg.py architecture_diagram.svg
	"$(CHROME)" --headless=new --disable-gpu --hide-scrollbars --force-device-scale-factor=2 \
	  --window-size=1600,900 --screenshot=architecture_diagram.png "file://$(CURDIR)/architecture_diagram.svg" 2>/dev/null
	test -x .venv/bin/python || (python3 -m venv .venv && .venv/bin/pip install -q matplotlib)
	.venv/bin/python tools/plot_telemetry.py telemetry/knock_heavy_neg.csv telemetry_performance.png
	@echo; echo "Media written to $(CURDIR):"; ls -lh architecture_diagram.svg architecture_diagram.png telemetry_performance.png
