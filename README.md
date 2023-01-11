# CLOVER Lab innfos actuator python wrapper

This code wraps the innfos-cpp-sdk and provides some convenience functions for interacting with it.

See the `examples` folder for examples of use.

## Get the submodules

If you used git to retrieve this source code, you will also need to get the submodules.

`git submodule update --init --recursive` 

## Installation

This package can be installed via pip. Open a terminal at the directory of this README.md.

```
SOURCE_PATH=$(pwd)
pip3 install $SOURCE_PATH
```


## Python binding for innfos-cpp-sdk

To recreate the binding with different configuration: edit `binder.config` and run `./make_binding.sh`