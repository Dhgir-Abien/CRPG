# CRPG
Improving Constrained Random Pattern Generation  by Automatic Design Abstraction

![CRPG](https://i.imgur.com/8SpLbVU.png)



## About CRPG

With the rapid growth in the scale and complexity of integrated circuits, functional verification has become increasingly important in the IC design flow. Simulation-based Constrained random methods can effectively handle the exponential growth of large-scale designs. Therefore, it has become the mainstream verification method in the industry. 

However, its high throughput and uniform distribution in probability make the generated patterns unplanned and purposeless. The simulator can only explore by random walk in the design state space. 

CPRG is a constrained random pattern generator written in C++, running on the fastest SystemVerilog simulation platform [Verilator](https://www.veripool.org/verilator/). It proposes a design abstraction technique to provide an abstract model of the DUV and counterexamples, combined with refinement techniques to systematically supplement it with the necessary details related to verification goals. This "big picture" model guides the generator to produce more purposeful patterns.

## Installation

If you want to run the testcase, you will need:

- Python 3.7 or later
- A C++14 compatible compiler, such as clang
- [Verilator](https://verilator.org/guide/latest/install.html) 4.2 or later
- [Yosys](https://github.com/YosysHQ/yosys#installation) 0.2 or later
- [Symbiyosys](https://yosyshq.readthedocs.io/projects/sby/en/latest/install.html#installing-from-source)
- [Boolector]([https://boolector.github.io](https://boolector.github.io/))
- [Yices2](https://github.com/SRI-CSL/yices2)

## Getting Started

1. Run CRPG on testcase ack1
   `make case-ack1`

2. Run BMC on testcase ack1

   `cd test/case-ack1; make bmc` 

3. Run K-Induction on testcase ack1
   `cd test/case-ack1; make ki` 

4. Run PDR on testcase ack1
   `cd test/case-ack1; make pdr` 

   

   

   
