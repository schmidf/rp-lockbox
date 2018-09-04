# Python GUI
This folder contains a python module ([rp_lockbox.py](rp_lockbox.py)) and a GUI application for
controlling the lockbox via SCPI.

A standalone executable version of the GUI for Windows can be downloaded
[here](https://github.com/schmidf/rp-lockbox/releases).

## System Requirements
The module and the GUI application require [Python 3](https://www.python.org/). Python package
dependencies of the GUI are managed using [pipenv](https://docs.pipenv.org/).

## Installation
The python module uses only the standard library and can be used without further setup.

For the GUI use pipenv to generate a new virtualenv and install the required packages:
```
cd examples/python
pipenv install
```

## Usage
Refer to the [SCPI command documentation](../../doc/SCPI_commands.rst) for an overview of the
available functions and parameters.

Use pipenv to activate the virtualenv and launch the GUI application:
```
pipenv run python gui.py
```
