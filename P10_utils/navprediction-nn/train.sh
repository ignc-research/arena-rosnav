#!/usr/bin/env bash
set -euo pipefail

poetry run python main.py \
  --train \
  --data_root "$HOME/data/nav.x150" \
  --checkpoint_dir "$HOME/data/nav/checkpoints" \
  --log_level INFO \
  --num_workers 4 \
  --batch_size 20 \
  --epochs 150