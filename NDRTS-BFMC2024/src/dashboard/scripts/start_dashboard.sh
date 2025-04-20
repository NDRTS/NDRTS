#!/bin/bash

SESSION=dashboard_session

if ! tmux has-session -t $SESSION 2>/dev/null; then
  tmux new-session -d -s $SESSION "cd /home/jetson/Desktop/NDRTS/NDRTS-BFMC2024/src/dashboard && npm start"
else
  echo "⚠️ tmux session $SESSION already exists. Not starting again."
fi
