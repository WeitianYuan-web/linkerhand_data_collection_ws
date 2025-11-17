
import re
import time
from threading import Thread
import subprocess

class TrainControl:
    def __init__(self):
        self.progress = 0  # 当前进度百分比
        self.current_step = 0  # 当前步骤
        self.total_steps = 0  # 总步骤
        self.task_name = None
        self.id = None

    def output_reader(self, process):
        """解析进度信息的线程函数"""
        progress_pattern = re.compile(r'(\d+)%\|.*?\| (\d+)/(\d+)')
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                print(output.strip(), flush=True)  # 原始输出
                # 解析进度条
                match = progress_pattern.search(output)
                if match:
                    self.progress = int(match.group(1))
                    self.current_step = int(match.group(2))
                    self.total_steps = int(match.group(3))

    def run_command(self, cmd,task_name=None,request_id=None):
        self.task_name = task_name
        self.id = request_id
        """启动命令并返回进程对象"""
        process = subprocess.Popen(
            cmd.split(),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        thread = Thread(target=self.output_reader, args=(process,))
        thread.daemon = True
        thread.start()
        return process
