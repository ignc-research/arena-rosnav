SHELL := /usr/bin/env bash
.SHELLFLAGS := -eu -o pipefail -c

.PHONY: default
default: test

.PHONY: test
test:
	poetry run pytest

.PHONY: train
train:
	./train.sh

.PHONY: analyze
analyze:
	poetry run python main.py \
	--analyze \
	--data_root "$(HOME)/data/nav.x150" \
	--log_level INFO \
	--epochs 5 \
	--num_workers 0 \
	--batch_size 20

.PHONY: install
install:
	@echo "Installing dependencies..."
	@poetry install