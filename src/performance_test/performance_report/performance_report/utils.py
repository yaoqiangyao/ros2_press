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

import argparse
import os
import sys

import pandas as pd

from performance_report.qos import DURABILITY, HISTORY, RELIABILITY


class ExperimentConfig:
    def __init__(
        self,
        com_mean: str = 'rclcpp-single-threaded-executor',
        process_configuration: str = 'INTRA_PROCESS',
        execution_strategy: str = 'INTER_THREAD',
        sample_transport: str = 'BY_COPY',
        msg: str = 'Array1k',
        pubs: int = 1,
        subs: int = 1,
        rate: int = 100,
        reliability: RELIABILITY = RELIABILITY.RELIABLE,
        durability: DURABILITY = DURABILITY.VOLATILE,
        history: HISTORY = HISTORY.KEEP_LAST,
        history_depth: int = 16,
        rt_prio: int = 0,
        rt_cpus: int = 0,
        prevent_cpu_idle: bool = False,
        max_runtime: int = 30,
        ignore_seconds: int = 5,
    ) -> None:
        self.com_mean = str(com_mean)
        self.process_configuration = str(process_configuration)
        self.execution_strategy = str(execution_strategy)
        self.sample_transport = str(sample_transport)
        self.msg = str(msg)
        self.pubs = int(pubs)
        self.subs = int(subs)
        self.rate = int(rate)
        self.reliability = RELIABILITY(reliability)
        self.durability = DURABILITY(durability)
        self.history = HISTORY(history)
        self.history_depth = int(history_depth)
        self.rt_prio = rt_prio
        self.rt_cpus = rt_cpus
        self.prevent_cpu_idle = prevent_cpu_idle
        self.max_runtime = max_runtime
        self.ignore_seconds = ignore_seconds

    def __eq__(self, o: object) -> bool:
        same = True
        same = same and self.com_mean == o.com_mean
        same = same and self.process_configuration == o.process_configuration
        same = same and self.execution_strategy == o.execution_strategy
        same = same and self.sample_transport == o.sample_transport
        same = same and self.msg == o.msg
        same = same and self.pubs == o.pubs
        same = same and self.subs == o.subs
        same = same and self.rate == o.rate
        same = same and self.reliability == o.reliability
        same = same and self.durability == o.durability
        same = same and self.history == o.history
        same = same and self.history_depth == o.history_depth
        same = same and self.rt_prio == o.rt_prio
        same = same and self.rt_cpus == o.rt_cpus
        same = same and self.prevent_cpu_idle == o.prevent_cpu_idle
        same = same and self.max_runtime == o.max_runtime
        same = same and self.ignore_seconds == o.ignore_seconds
        return same

    def log_file_name(self) -> str:
        if self.process_configuration == 'INTRA_PROCESS':
            return self.log_file_name_intra()
        else:
            return self.log_file_name_sub()

    def write_log_file_name(self, com_mean_suffix: str) -> str:
        params = [
            self.com_mean,
            str(self.process_configuration) + com_mean_suffix,
            self.execution_strategy,
            self.sample_transport,
            self.msg,
            self.pubs,
            self.subs,
            self.rate,
            self.reliability,
            self.durability,
            self.history,
            self.history_depth,
            self.rt_prio,
            self.rt_cpus,
            self.prevent_cpu_idle,
        ]
        str_params = map(str, params)
        return '_'.join(str_params) + '.json'

    def log_file_name_intra(self) -> str:
        return self.write_log_file_name(com_mean_suffix='')

    def log_file_name_pub(self) -> str:
        return self.write_log_file_name(com_mean_suffix='-pub')

    def log_file_name_sub(self) -> str:
        return self.write_log_file_name(com_mean_suffix='-sub')

    def as_dataframe(self) -> pd.DataFrame:
        return pd.DataFrame({
            'com_mean': self.com_mean,
            'process_configuration': self.process_configuration,
            'execution_strategy': self.execution_strategy,
            'sample_transport': self.sample_transport,
            'msg': self.msg,
            'pubs': self.pubs,
            'subs': self.subs,
            'rate': self.rate,
            'reliability': self.reliability,
            'durability': self.durability,
            'history': self.history,
            'history_depth': self.history_depth,
            'rt_prio': self.rt_prio,
            'rt_cpus': self.rt_cpus,
            'prevent_cpu_idle': self.prevent_cpu_idle,
            'max_runtime': self.max_runtime,
            'ignore_seconds': self.ignore_seconds,
        }, index=[0])

    def get_members(self) -> list:
        members = []
        for attribute in dir(self):
            if not callable(getattr(self, attribute)) \
              and not attribute.startswith('__'):
                members.append(attribute)
        return members

    def cli_commands(self, perf_test_exe_cmd, output_dir) -> list:
        args = self.cli_args(output_dir)
        commands = []

        if len(args) == 1:
            commands.append(perf_test_exe_cmd + args[0])
        elif len(args) == 2:
            sub_args, pub_args = args
            commands.append(perf_test_exe_cmd + sub_args + ' &')
            commands.append('sleep 1')
            commands.append(perf_test_exe_cmd + pub_args)
            commands.append('sleep 5')
        else:
            raise RuntimeError('Unreachable code')

        return commands

    def cli_args(self, output_dir) -> list:
        args = ''
        args += f' -c {self.com_mean}'
        args += f' -e {self.execution_strategy}'
        if self.sample_transport == 'SHARED_MEMORY':
            args += ' --shared-memory'
        if self.sample_transport == 'LOANED_SAMPLES':
            args += ' --shared-memory --loaned-samples'
        args += f' -m {self.msg}'
        args += f' -r {self.rate}'
        if self.reliability == RELIABILITY.RELIABLE:
            args += ' --reliability RELIABLE'
        else:
            args += ' --reliability BEST_EFFORT'
        if self.durability == DURABILITY.TRANSIENT_LOCAL:
            args += ' --durability TRANSIENT_LOCAL'
        else:
            args += ' --durability VOLATILE'
        if self.history == HISTORY.KEEP_LAST:
            args += ' --history KEEP_LAST'
        else:
            args += ' --history KEEP_ALL'
        args += f' --history-depth {self.history_depth}'
        args += f' --use-rt-prio {self.rt_prio}'
        args += f' --use-rt-cpus {self.rt_cpus}'
        if self.prevent_cpu_idle:
            args += ' --prevent-cpu-idle'
        args += f' --max-runtime {self.max_runtime}'
        args += f' --ignore {self.ignore_seconds}'
        if self.process_configuration == 'INTRA_PROCESS':
            args += f' -p {self.pubs} -s {self.subs}'
            args += f' --logfile {os.path.join(output_dir, self.log_file_name_intra())}'
            return [args]
        else:
            args_sub = args + f' -p 0 -s {self.subs} --expected-num-pubs {self.pubs}'
            args_sub += f' --logfile {os.path.join(output_dir, self.log_file_name_sub())}'
            args_pub = args + f' -s 0 -p {self.pubs} --expected-num-subs {self.subs}'
            args_pub += f' --logfile {os.path.join(output_dir, self.log_file_name_pub())}'
            return [args_sub, args_pub]


class LineConfig:
    def __init__(
        self,
        style: str = 'solid',
        width: int = 2,
        alpha: float = 1.0
    ) -> None:
        self.style = style
        self.width = width
        self.alpha = alpha


class MarkerConfig:
    def __init__(
        self,
        shape: str = 'dot',
        size: int = 25,
        alpha: float = 1.0
    ) -> None:
        self.shape = str(shape)
        self.size = size
        self.alpha = alpha


class ThemeConfig:
    def __init__(
        self,
        color: str = '#0000ff',
        marker: MarkerConfig = MarkerConfig(),
        line: LineConfig = LineConfig(),
    ) -> None:
        self.color = str(color)

        if type(marker) is MarkerConfig:
            self.marker = marker
        if type(marker) is dict:
            self.marker = MarkerConfig(**marker)

        if type(line) is LineConfig:
            self.line = line
        if type(line) is dict:
            self.line = LineConfig(**line)


class DatasetConfig:
    def __init__(
        self,
        name: str = 'default_dataset',
        theme: ThemeConfig = ThemeConfig(),
        experiments: [ExperimentConfig] = [ExperimentConfig()],
        headers: [(str, dict)] = ['default_experiment', {}],
        dataframe: pd.DataFrame = pd.DataFrame(),
    ) -> None:
        self.name = name
        self.theme = theme
        self.experiments = experiments
        self.headers = headers
        self.dataframe = dataframe


class FileContents:
    def __init__(self, header: dict, dataframe: pd.DataFrame) -> None:
        self.header = header
        self.dataframe = dataframe


DEFAULT_TEST_NAME = 'experiments'


class PerfArgParser(argparse.ArgumentParser):

    def init_args(self):
        self.add_argument(
            '--log-dir',
            '-l',
            default='.',
            help='The directory for the perf_test log files and plot images',
        )
        self.add_argument(
            '--test-name',
            '-t',
            default=DEFAULT_TEST_NAME,
            help='Name of the experiment set to help give context to the test results',
        )
        self.add_argument(
            '--configs',
            '-c',
            default=[],
            nargs='+',
            help='The configuration yaml file(s)',
        )
        self.add_argument(
            '--perf-test-exe',
            default='ros2 run performance_test perf_test',
            help='The command to run the perf_test executable',
        )
        self.add_argument(
            '--force',
            '-f',
            action='store_true',
            help='Force existing results to be overwritten (by default, they are skipped).',
        )

        if len(sys.argv) == 1:
            print('[ERROR][ %s ] No arguments given\n' % self.prog)
            self.print_help()
            sys.exit(2)
        elif (sys.argv[1] == '-h' or sys.argv[1] == '--help'):
            self.print_help()
            sys.exit(0)

    def error(self, msg):
        print('[ERROR][ %s ] %s\n' % (self.prog, msg))
        self.print_help()
        sys.exit(2)

    def exit(self, status=0, message=None):  # noqa: A003
        if status:
            raise Exception(f'Exiting because of an error: {message}')
        self.print_help()
        exit(status)


class cliColors:
    ESCAPE = '\033'
    GREEN = ESCAPE + '[92m'
    WARN = ESCAPE + '[93m'
    ERROR = ESCAPE + '[91m'
    ENDCOLOR = ESCAPE + '[0m'


def colorString(raw_string, color_type) -> str:
    return color_type + raw_string + cliColors.ENDCOLOR


def colorPrint(raw_string, color_type):
    print(colorString(raw_string, color_type))


def create_dir(dir_path) -> bool:
    try:
        os.makedirs(dir_path)
        return True
    except FileExistsError:
        print(colorString('Log directory already exists', cliColors.WARN))
    except FileNotFoundError:
        # given path is not viable
        return False
