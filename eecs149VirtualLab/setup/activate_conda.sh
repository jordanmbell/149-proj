#!/bin/bash

__conda_setup="$('/home/eecs149/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/eecs149/miniconda3/etc/profile.d/conda.sh" ]; then
        . "/home/eecs149/miniconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/eecs149/miniconda3/bin:$PATH"
    fi
fi
unset __conda_setup