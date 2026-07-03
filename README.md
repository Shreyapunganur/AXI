# AXI
I built a 2-master, 2-slave AXI4 crossbar interconnect with clock domain crossing (CDC) support. The idea was to implement something close to what actually sits inside a real SoC between the CPU, DMA engine, and memory peripherals.

The interconnect routes transactions based on address decoding and handles the case where two masters want the same slave simultaneously using QoS-based arbitration. Slave 1 is on a different clock (83 MHz vs 100 MHz) and the data crosses through a gray-code async FIFO bridge.

On the verification side I wrote SVA protocol checkers, functional coverage, a scoreboard with a shadow memory reference model, and constrained-random stimulus. Most of the interesting bugs showed up during the random phase.
