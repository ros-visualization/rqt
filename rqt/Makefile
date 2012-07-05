include $(shell rospack find mk)/cmake_stack.mk

test:
  # requires installation of nose from pypi
	$(shell find . -maxdepth 1 -mindepth 1 -type d | xargs nosetests )
