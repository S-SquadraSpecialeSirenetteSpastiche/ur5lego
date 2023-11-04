#!/bin/bash

# Publish a message to the /connect_links topic
rostopic pub /connect_links ur5lego/ConnectLinks "{
    'model1': 'ur5',
    'link1': 'wrist_3_link', 
    'model2': 'block1', 
    'link2': 'block', 
    'connect': True
}" -1