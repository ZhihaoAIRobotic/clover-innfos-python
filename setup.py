from glob import glob
from setuptools import setup
import sysconfig
from pybind11.setup_helpers import Pybind11Extension
import os


MODULE_NAME = "clover_ActuatorController"
BASE_DIR = os.path.dirname(__file__)
binding_source_files_string = open(os.path.join(BASE_DIR,f"binding/{MODULE_NAME}.sources"),'r').read()
binding_source_files = binding_source_files_string.strip().split('\n')
binding_source_files = ["binding/"+pth for pth in binding_source_files]

# Copy libActuatorController.so into the package directory
import shutil
shutil.copyfile(os.path.join(BASE_DIR,"innfos-cpp-sdk/sdk/lib/linux_x86_64/libActuatorController.so"), os.path.join(BASE_DIR,'clover_innfos_python','libActuatorController.so'))

ext_modules = [
    Pybind11Extension(
        MODULE_NAME,
        sorted(binding_source_files),  # Sort source files for reproducibility
        include_dirs=["innfos-cpp-sdk/sdk/include"],
        libraries=["ActuatorController"],
        library_dirs=["innfos-cpp-sdk/sdk/lib/linux_x86_64/"],
        extra_link_args=["-Wl,-rpath,$ORIGIN:$ORIGIN/clover_innfos_python/"], # Needed so libActuatorController.so can be found by clover_ActuatorController
    ),
]

setup(
    name='clover_innfos_python',
    packages=['clover_innfos_python'],
    setup_requires = ['pybind11','setuptools-git-versioning<2','setuptools_scm'],
    install_requires = ["matplotlib", "scipy", "pandas", "numpy"],
    provides = ["clover_innfos_python"],
    ext_modules=ext_modules,
    package_data={'clover_innfos_python': ['libActuatorController.so']},
    use_scm_version=True,
)

