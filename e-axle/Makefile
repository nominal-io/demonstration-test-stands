.PHONY: test

test:
	uv run python -m pytest tests/ -v
	uvx pyright --pythonpath .venv/bin/python3 stand.py stand_config.py channels.py units.py
