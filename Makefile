init:
	python3.9 -m pip install -r requirements.txt

test:
	py.test tests

syntax:
	pylint *

run:
	python3.9 main.py

.PHONY: init test syntax
