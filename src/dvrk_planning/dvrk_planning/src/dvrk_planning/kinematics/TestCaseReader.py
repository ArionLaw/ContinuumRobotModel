import numpy as np
import os
import sys
import re

#----------------------------------------------------------------------------------------------------------------------------------------------#
### Reading Test Case .txt File ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def read_TestCaseFile(input_filename):
    file_path = sys.path[0]+ "/test_cases/" + input_filename

    try:
        with open(file_path, 'r') as file:
            content = file.read()

            # Define patterns to capture values
            input_current_output_js_pattern = re.compile(r'self\.input_current_output_js:\s+\[(.*?)\]',re.DOTALL)
            absolute_output_tf_pattern = re.compile(r'absolute_output_tf:\s+\[\[(.*?)\]\]', re.DOTALL)

            # Find all matches of the patterns in the content
            input_current_output_js_matches = input_current_output_js_pattern.findall(content)
            absolute_output_tf_matches = absolute_output_tf_pattern.findall(content)

            # Extract and convert all logged str representation lists into a list of arrays and matrix
            # For list of list 
            input_current_output_js_list = []
            for log_entry in input_current_output_js_matches:
                configurations = log_entry.split()
                float_joint_values = [] 
                for values in configurations:
                    float_joint_values.append(float(values))
                input_current_output_js_list.append(float_joint_values)
            # For list of matrix
            tf_matrices_list = []
            for log_entry in absolute_output_tf_matches:
                matrix = []
                matrix_rows = log_entry.split('\n')
                for row in matrix_rows:
                    row = row.strip().lstrip('[').rstrip(']')
                    float_row = [] # holds 4 float values
                    for num in row.split():
                            float_row.append(float(num))
                    matrix.append(float_row)
                tf_matrices_list.append(matrix)

            return input_current_output_js_list,tf_matrices_list
            #print(input_current_output_js_list[0])
            #print(tf_matrices_list[0])

    except FileNotFoundError:
        print(f"The file at {file_path} was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

