#!/bin/bash

# 2. 检查输入参数是否为 Main 或 Relay，默认为 Main
MODE="${1:-Main}"

# 根据输入参数设置模式选项，并输出当前启动的模式
if [[ "$MODE" == "Relay" ]]; then
        echo "启动模式: Relay"
        MODE_OPTION="Relay"
elif [[ "$MODE" == "Main" ]]; then
        echo "启动模式: Main"
        MODE_OPTION="Main"
else
        echo "启动模式: None"
        MODE_OPTION="None"
fi

# 3. 读取或设置默认的 rate 和 max-runtime 参数
RATE="${2:-30}"
MAX_RUNTIME="${3:-600}"

# 输出当前的 rate 和 max-runtime
echo "速率: $RATE"
echo "最大运行时间: $MAX_RUNTIME"

# 4. 定义主题数量
NUM_TOPICS=1

# 追踪后台进程的 PID 数组
declare -a PIDS=()

# 5. 循环启动每个主题的性能测试
for i in $(seq 1 "$NUM_TOPICS"); do
        # 构建主题名称
        TOPIC_NAME="hello_${i}_"

        # 构建并打印性能测试命令
        COMMAND="/opt/performance_test/lib/performance_test/perf_test \
        --rate $RATE \
        --topic $TOPIC_NAME \
        --max-runtime $MAX_RUNTIME \
        --logfile /report/report_${TOPIC_NAME}${MODE_OPTION}.csv \
        --roundtrip-mode $MODE_OPTION \
        --reliability RELIABLE --history-depth 10000"

        echo "启动性能测试: $COMMAND"

        # 启动性能测试进程，并捕获其 PID
        eval "$COMMAND" &
        PIDS+=($!) # 保存进程 PID
done

# 6. 等待所有后台进程完成
for PID in "${PIDS[@]}"; do
        wait "$PID"
done

echo "所有主题的性能测试已完成。"
