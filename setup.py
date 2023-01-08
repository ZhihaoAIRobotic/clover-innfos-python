from glob import glob
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension
import os


MODULE_NAME = "clover_ActuatorController"
binding_source_files_string = open(os.path.join(os.path.dirname(__file__),f"binding/{MODULE_NAME}.sources"),'r').read()
binding_source_files = binding_source_files_string.strip().split('\n')
binding_source_files = ["binding/"+pth for pth in binding_source_files]

ext_modules = [
    Pybind11Extension(
        MODULE_NAME,
        sorted(binding_source_files),  # Sort source files for reproducibility
        include_dirs=["innfos-cpp-sdk/sdk/include"],
        libraries=["ActuatorController"],
        library_dirs=["innfos-cpp-sdk/sdk/lib/linux_x86_64/"],
        extra_link_args=["-Wl,-rpath,$ORIGIN"], # Needed so libActuatorController.so can be found by clover_ActuatorController
    ),
]

setup(
    name='clover_innfos_python',
    packages=['clover_innfos_python'],
    setup_requires = ['pybind11','setuptools-git-versioning<2','setuptools_scm'],
    install_requires = ["matplotlib", "scipy", "pandas", "numpy"],
    provides = ["clover_innfos_python"],
    ext_modules=ext_modules,
    data_files=[('',['innfos-cpp-sdk/sdk/lib/linux_x86_64/libActuatorController.so'])],
    #setuptools_git_versioning={"enabled": True,},
    use_scm_version=True,
)

