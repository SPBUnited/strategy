import os
import typing

import attr
import matlab
import matlab.engine
import numpy as np

from bridge.common import config


@attr.s(auto_attribs=True)
class MatlabEngine:

    engine: typing.Any = attr.ib(init=False)

    def __attrs_post_init__(self):
        self.engine = matlab.engine.start_matlab()
        matlab_main_dir = config.MATLAB_SCRIPTS_PATH
        self.engine.addpath(matlab_main_dir)
        for path in os.listdir(matlab_main_dir):
            path = os.path.join(matlab_main_dir, path)
            if os.path.isdir(path):
                self.engine.addpath(os.path.join(matlab_main_dir, path), nargout=0)

    def run_function(self, function_name: str, *args) -> typing.Optional[np.ndarray]:
        converted_args = [matlab.double(arg) for arg in args]
        engine_function = getattr(self.engine, function_name)
        result = engine_function(*converted_args)
        if result:
            return np.array(result)
        else:
            return None


matlab_engine = MatlabEngine()
