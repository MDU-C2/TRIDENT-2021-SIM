# Documentation

[TOC]

## Requirements

The easiest way to build the documentation in to create a Python virtual environment.

```bash
sudo apt install doxygen       # if you have not installed it yet
sudo apt install python3-venv  # if you have not installed it yet
python3 -m venv env
source env/bin/activate        # type 'desactivate' in the terminal to exit the venv
pip install sphinx sphinx_rtd_theme lxml
```

## Compile documentation

To compile the documentation just activate the virtual environment and execute makefile:

```bash
source env/bin/activate
cd doc
make
```

Documentation will be available in `doc/html`

```bash
cd html
python3 -m http.server
```
