#!/usr/bin/env python

# Copyright (c) 2016, Socially Intelligent Machines Lab 
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of data_logger_bag nor the names of its 
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Vivian Chu, vchu@gatech.edu

'''
load_h5_dataset.py

Script to start loading data into pytables and convert into meaningful features
'''
import rospy
import sys
import tables
import numpy as np
import cPickle
from collections import defaultdict
import matplotlib.pyplot as plt

def pull_groups(group_data, group_name):

    # Pull out all group names if exists
    group_fields = [_v for _v in group_data._v_groups]
    fields = [_v for _v in group_data._v_leaves]
    
    data_dict = dict()
    if len(group_fields) > 0:
       
        for new_group_name in group_fields: 

            new_group_data = eval('group_data.'+new_group_name)
            data_dict[new_group_name] = pull_groups(new_group_data, new_group_name) 
   
    # Base case where we stop and store if we no longer have groups left
    if len(fields) > 0:

        # Pull out fields for the topic
        for field in fields:
            data_dict[field] = eval('group_data.'+field+'[:]')
        

    return data_dict

def extract_data(one_run):
    '''
    Pulls out the data we want from a single run
    '''
    # Create a dictionary to store all of the run
    store_data = defaultdict(dict)

    # pull out the topics
    data_topics = [_v for _v in one_run._v_groups]

    for topic in data_topics:

        # Pull out the data
        topic_data = eval('one_run.'+topic)
        store_data[topic] = pull_groups(topic_data, topic)

    return store_data

def load_data_section(data, directories, searching, max_level=None, spec_key=False):

    # Check if we're looking for the start point of extraction
    if searching:
        stored_data = dict()
        topics = [_v for _v in data._v_groups]

        # check how deep to go if specified (for speedup purposes)
        if max_level is not None:
            if max_level < 0:
                return ({}, False)
            else:
                max_level-=1

        if len(topics) < 1:
            return ({}, False)
       
        # Check if the first value in directories is on this level
        if directories[0] not in topics:
            for topic in topics: 
                data_next = eval('data.'+topic) 
                (ret_dict, found) = load_data_section(data_next, directories, True, max_level=max_level, spec_key=spec_key)
                if found:
                    stored_data[topic] = ret_dict               
           
            return (stored_data, len(stored_data.keys()) > 0)  
        else:
            return (load_data_section(data, directories, False, max_level=max_level, spec_key=spec_key), True)
    else:
        stored_data = dict()
        dir_levels = list(directories)
        # Go through each group until we get down to the keys
        # Makes the assumption that directories do not have data other
        # than when we get to the end of the directories given
        while dir_levels:
            
            # Pull out the data
            topics = [_v for _v in data._v_groups]
            loc = dir_levels.pop(0)
            
            # Check if we are getting the right directory structure
            # If not, then pull out all values up until
            if loc not in topics:
                print('Warning: directory levels passed in do not match actual h5 structure')
                print('dir value: %s , structure is: %s'% (loc, topics))
                print('Skipping this branch')
                return stored_data

            data = eval('data.'+loc) 

        # Now start storing
        # Check if we've specified a specific key - if so, we're only extracting one run
        if spec_key:
            stored_data[data._v_name] = extract_data(data)
        else:
            for single_run in data: 
                stored_data[single_run._v_name] = extract_data(single_run) 

        # Store the data away with the directory structure intact    
        segment_data = dict()
        create_dict_recurse(list(directories), segment_data, stored_data)

        return segment_data




def create_dict_recurse(keys, cur_dict, stored_data):
    if len(keys) > 1:
        new_dict = cur_dict.setdefault(keys.pop(0), {})
        create_dict_recurse(keys, new_dict, stored_data)
    else:
        new_dict = cur_dict.setdefault(keys.pop(0), stored_data)

def load_data(input_filename, output_filename, save_to_file, 
              directories=None, max_level=None, spec_key=False):

    if not input_filename.endswith(".h5"):
        raise Exception("Input file is %s \nPlease pass in a hdf5 data file" % input_filename)

    if save_to_file: 
        if not output_filename.endswith(".pkl"):
            output_filename = output_filename + '.pkl'

    # Load the data from an h5 file
    all_data = tables.open_file(input_filename)
    
    # Much faster way to access the root node
    root_data = all_data.get_node('/')

    if directories is not None:
        (stored_data, done) = load_data_section(root_data, directories, True, max_level=max_level, spec_key=spec_key)
    else:
        # Create a dictionary to store all of the data 
        stored_data = dict()

        # Pull pointers to only the file heads of the data structure
        # Very slow method - instead we just group the name and evaluate
        #all_runs_root = [_g for _g in all_data.walkGroups("/") if _g._v_depth == 1]
        all_runs_root = [_g for _g in all_data.get_node('/')._v_groups]

        # For each file extract the segments and data
        for _objectRunName in all_runs_root:
            _objectRun = eval('root_data.'+_objectRunName) 
            stored_data[_objectRun._v_name] = extract_data(_objectRun)

    # if we want to save to file
    if (save_to_file):
        file_ptr = open(output_filename, "w")
        cPickle.dump(stored_data, file_ptr, cPickle.HIGHEST_PROTOCOL)
        file_ptr.close()
   
    # Close the h5 file
    all_data.close()

    return stored_data

def main():

    if len(sys.argv) == 2:
        input_file = sys.argv[1]
        data = load_data(input_file, '', False)

        import pdb; pdb.set_trace()
    elif len(sys.argv) == 3:
        input_file = sys.argv[1]
        output_file = sys.argv[2]

        load_data(input_file, output_file, True)
    else:
        raise Exception("Usage: %s [input_file] [output_file]", sys.argv[0])

if __name__== "__main__":
    main()

