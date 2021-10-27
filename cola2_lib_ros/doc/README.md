# Documentation

[TOC]

## Requirements

The easiest way to build the documentation in to create a Python virtual environment.

```bash
sudo apt install doxygen       # if you have not installed it yet
python2 -m virtualenv env --system-site-packages  # need to go through code and ROS is still python2
# source ros here
source env/bin/activate        # type 'desactivate' in the terminal to exit the venv
pip install sphinx sphinx_rtd_theme
```

## Compile documentation

To compile the documentation just activate the virtual environment and execute makefile:

```bash
# source ros here if you have not done it yet
source env/bin/activate
cd doc
make
```

Documentation will be available in `doc/html`

```bash
cd html
python3 -m http.server
```
