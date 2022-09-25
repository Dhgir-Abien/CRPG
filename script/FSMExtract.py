#!/usr/bin/env python3

import re
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('design_under_test_log')
parser.add_argument('control_template')
parser.add_argument('control_hpp')
args = parser.parse_args()

pattern_variable = re.compile(r"\s*Information on FSM .*\(\\(.*)\):")
pattern_encoding = re.compile(r"\s*(\d+):\s*\d+'([0|1]+).*")
pattern_transition = re.compile(r"\s*(\d+):\s*(\d+).*->\s+(\d+).*")
state_variable = []
state_encoding = []
state_transition = []

with open(yosys_design_log_path) as yosys_design_log_f:
    while True:
        line = yosys_design_log_f.readline()
        if(not line):
            break
        else:
            if(line.find('Executing FSM_INFO pass (dumping all available information on FSM cells).') != -1):
                break

    while True:
        line = yosys_design_log_f.readline()
        if(not line):
            break

        elif(line.find('Information on FSM') != -1):
            m = pattern_variable.match(line)
            state_variable.append(m.group(1))
        
        elif(line.find('State encoding:') != -1):
            tempList = []
            line = yosys_design_log_f.readline()
            while(True):
                m = pattern_encoding.match(line)
                if(m == None):
                    break
                tempList.append( (m.group(1), m.group(2)) )
                line = yosys_design_log_f.readline()
            state_encoding.append(tempList)

        elif(line.find('Transition Table (state_in, ctrl_in, state_out, ctrl_out):') != -1):
            tempSet = set()
            line = yosys_design_log_f.readline()
            while(True):
                m = pattern_transition.match(line)
                if(m == None):
                    break
                tempSet.add( (m.group(2), m.group(3)) )
                line = yosys_design_log_f.readline()
            state_transition.append(tempSet)

#print(state_variable)
#print(state_encoding)
#print(state_transition)

dut_control_template = []
with open(dut_control_template_path) as dut_control_template_f:
    dut_control_template = dut_control_template_f.readlines()

with open(dut_control_hpp_path, 'w') as dut_control_hpp_f:
    for line in dut_control_template:
        dut_control_hpp_f.write(line)
        
        if(line.find('// State Inst') != -1):
            for i in range(0, len(state_variable)):
                dut_control_hpp_f.write('\t\tfsm_list.emplace(fsm_list.end(), {}, \"{}\");\n'.format(i, state_variable[i]))
                for state in state_encoding[i]:
                    dut_control_hpp_f.write("\t\tfsm_list[{}].states.emplace(fsm_list[{}].states.end(),\n".format(i, i))
                    dut_control_hpp_f.write("\t\t\t\t\t\t\t\t\t{},\n".format(state[0]))
                    dut_control_hpp_f.write("\t\t\t\t\t\t\t\t\t0b{},\n".format(state[1]))
                    dut_control_hpp_f.write("\t\t\t\t\t\t\t\t\tstd::vector<unsigned>{")
                    flag = True
                    for tran in state_transition[i]:
                        if(tran[0] == state[0]):
                            if(not flag):
                                dut_control_hpp_f.write(", {}".format(tran[1]));
                            else:
                                dut_control_hpp_f.write("{}".format(tran[1]));
                                flag = False
                    dut_control_hpp_f.write("}\n\t\t);\n")
                dut_control_hpp_f.write("\n")
                    
print('{} is generated.'.format(dut_control_hpp_path))
