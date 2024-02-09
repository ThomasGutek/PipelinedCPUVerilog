# PipelinedCPUVerilog
5 stage pipelined MIPS CPU in Verilog

## Overview
This project implements a 5-stage pipelined CPU in Verilog. The CPU architecture follows the classic RISC pipeline structure, consisting of the following stages: Instruction Fetch (IF), Instruction Decode (ID), Execute (EX), Memory Access (MEM), and Write Back (WB). Pipelining is employed to enhance performance by allowing multiple instructions to be processed simultaneously.

## Features
Five pipeline stages: IF, ID, EX, MEM, WB.
Support for basic RISC instructions including arithmetic, logical, load/store, and branch instructions.
Forwarding and hazard detection mechanisms to handle data hazards.
Instruction and data memory units.
Control unit to manage instruction execution and pipeline control signals.
Instruction and data caches for improved performance.

## File Structure
The project files are organized as follows:

Datapath.v: Contains the modules for which the program executes
Testbench.v: Runs the Datapath with all possible input values.

## Simulation and Synthesis
Use a Verilog simulator, I used Xlinix Vivado for simulation and synthesis, to simulate the CPU's behavior. Compile all Verilog files and run the simulation testbench to observe CPU operation and performance. Synthesize the Verilog code using a synthesis tool to generate a gate-level netlist. Perform timing analysis and optimize for target FPGA or ASIC technology.

## Testing
Comprehensive testbenches are provided to verify the functionality and performance of the CPU design.
Test various instruction sequences to ensure correct instruction execution and pipeline operation.
Verify handling of data hazards, control hazards, and corner cases.

## Dependencies
This project requires a Verilog simulator and synthesizer, like Xlinix Vivado, for simulation and synthesis.
Ensure proper setup and configuration of the simulation environment and synthesis toolchain.

## Acknowledgements
This project was assigned by the Pennsylavnia State University's Computer architecture and assembly course. I received a 100%.

## Notes
The program requires Xlinix Vivado to run. My university covered the cost, but now that I am out of the class I am unable to run this project. However, you can still view the code.
