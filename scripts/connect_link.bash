#!/bin/bash

# Publish a message to the /connect_links topic
rostopic pub /connect_links ur5lego/ConnectLinks "{
    'model1': 'ur5',
    'link1': 'wrist_3_link', 
    'model2': '$1', 
    'link2': 'block', 
    'connect': $2
}" -1