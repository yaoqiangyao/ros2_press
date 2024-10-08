# Copyright 2022 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time

from performance_report.logs import getExperiments
from performance_report.utils import (cliColors,
                                      colorPrint,
                                      create_dir,
                                      ExperimentConfig,
                                      PerfArgParser)

import yaml


def run_experiment(cfg: ExperimentConfig, perf_test_exe_cmd, output_dir, overwrite: bool):
    lf = os.path.join(output_dir, cfg.log_file_name())
    if os.path.exists(lf) and not overwrite:
        formatted_string = \
            f'Skipping experiment {cfg.log_file_name()} as results already exist in ' + output_dir
        colorPrint(formatted_string, cliColors.WARN)
        return
    else:
        colorPrint(f'Running experiment {cfg.log_file_name()}', cliColors.GREEN)

    if cfg.process_configuration == 'INTRA_PROCESS':
        cli_args = cfg.cli_args(output_dir)[0]
        os.system(perf_test_exe_cmd + cli_args)
    else:
        cli_args_sub, cli_args_pub = cfg.cli_args(output_dir)
        os.system(perf_test_exe_cmd + cli_args_sub + ' &')
        time.sleep(1)
        os.system(perf_test_exe_cmd + cli_args_pub)


def run_experiments(files: 'list[str]', perf_test_exe_cmd, output_dir, overwrite: bool):
    # make sure output dir exists
    create_dir(output_dir)
    # loop over given run files and run experiments
    for run_file in files:
        with open(run_file, 'r') as f:
            run_cfg = yaml.load(f, Loader=yaml.FullLoader)

        run_configs = getExperiments(run_cfg['experiments'])

        for run_config in run_configs:
            run_experiment(run_config, perf_test_exe_cmd, output_dir, overwrite)


def main():
    parser = PerfArgParser()
    parser.init_args()
    args = parser.parse_args()
    log_dir = getattr(args, 'log_dir')
    test_name = getattr(args, 'test_name')
    run_files = getattr(args, 'configs')
    perf_test_exe_cmd = getattr(args, 'perf_test_exe')
    overwrite = bool(getattr(args, 'force'))

    log_dir = os.path.join(log_dir, test_name)
    run_experiments(run_files, perf_test_exe_cmd, log_dir, overwrite)


# if this file is called directly
if __name__ == '__main__':
    main()
