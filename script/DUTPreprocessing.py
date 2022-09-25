#!/usr/bin/env python3

import re
import json
import argparse
from os import listdir
from os.path import isfile, isdir, join

parser = argparse.ArgumentParser()
parser.add_argument('top')
parser.add_argument('in_dir')
parser.add_argument('out_dir')
parser.add_argument('dut_assertion_hpp')
parser.add_argument('dut_assertion_template')
args = parser.parse_args()

top = args.top
in_dir_path = args.in_dir
out_dir_path = args.out_dir
dut_assertion_template_path = args.dut_assertion_template
dut_assertion_hpp_path = args.dut_assertion_hpp

dpi_template = '''
// DPI Header
import "DPI-C" function void set_assert_flag (input int index);
import "DPI-C" function void set_constraint_flag (input int index);
import "DPI-C" function void set_stable_flag (input int index);

'''
dut_assert_str = '\t\tassertion.emplace(assertion.end(), "{}", {}, {}, "{}");\n'
pattern_property = re.compile(r".*assert\s*\((.*)\);")

print('Preprocessing {} ...'.format(top))

assert_count = 0
assert_list = []

files = listdir(in_dir_path)
for f in files:
    in_file_path = join(in_dir_path, f)
    if isfile(in_file_path) and in_file_path.endswith('.v'):
        f_v = []
        with open(in_file_path) as in_file_f:
            f_v = in_file_f.readlines()

        if(f == top+'.v'):
            flag = True
            out_file_path = join(out_dir_path, 'design_under_test.v')
        else:
            flag = False
            out_file_path = join(out_dir_path, f)

        with open(out_file_path, 'w') as out_file_path_f:
            for idx, line in enumerate(f_v):
                if(flag):
                    flag = False
                    out_file_path_f.write(dpi_template)
                    out_file_path_f.write(line)

                elif(line.find('assert') != -1):
                    m = pattern_property.match(line)
                    if(m != None):
                        line = line.replace('assert', "Assertion_{} : assert".format(assert_count))
                        line = line.replace(';', " else set_assert_flag({});".format(assert_count))
                        assert_list.append( (f, assert_count, idx, m.group(1)) )
                        assert_count += 1
                    out_file_path_f.write(line)

                else:
                    out_file_path_f.write(line)
        print('{} is generated.'.format(out_file_path))

dut_assertion_template = []
with open(dut_assertion_template_path) as dut_assertion_template_f:
    dut_assertion_template = dut_assertion_template_f.readlines()

with open(dut_assertion_hpp_path, 'w') as dut_assertion_hpp_f:
    for line in dut_assertion_template:
        dut_assertion_hpp_f.write(line)
        
        if(line.find('// Assert Init') != -1):
            for a in assert_list:
                dut_assertion_hpp_f.write( dut_assert_str.format(a[0], a[1], a[2]+1, a[3]) )
print('{} is generated.'.format(dut_assertion_hpp_path))
