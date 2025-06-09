#!/usr/bin/env python3
# 樹莓派 WebSocket Clients：自動 + 手動模式（支援動態切換）
# ===============================================
import asyncio
import websockets
import subprocess
import json
import os
import signal

ROS_LAUNCH_PROCESS = None
MODE = None
WS_SERVER_URI = "ws://172.20.10.2:8765"
ROS_SETUP_SCRIPT = "/mnt/USB_DISK/ros2_workspace/install/setup.bash"

COMMAND_MAP = {
    "auto": {
        "start": "start_auto",
        "stop": "stop_auto",
        "launch_file": "camera.launch.py",
        "package": "my_robot"
    },
    "manual": {
        "start": "start_robot",
        "stop": "shutdown_robot",
        "launch_file": "keybgggoard.launch.py",
        "package": "my_robot"
    }
}

async def launch_ros_process(mode):
    global ROS_LAUNCH_PROCESS
    mode_cfg = COMMAND_MAP[mode]
    if ROS_LAUNCH_PROCESS is None or ROS_LAUNCH_PROCESS.poll() is not None:
        print(f"🚀 啟動 {mode_cfg['launch_file']}")
        env = os.environ.copy()
        launch_cmd = (
            f"source {ROS_SETUP_SCRIPT} && "
            f"ros2 launch {mode_cfg['package']} {mode_cfg['launch_file']}"
        )
        ROS_LAUNCH_PROCESS = subprocess.Popen(
            ["bash", "-c", launch_cmd],
            env=env,
            preexec_fn=os.setsid
        )
    else:
        print(f"⚠️ {mode}_mode 已在執行中，略過啟動")

async def stop_ros_process(mode):
    global ROS_LAUNCH_PROCESS
    mode_cfg = COMMAND_MAP[mode]
    if ROS_LAUNCH_PROCESS and ROS_LAUNCH_PROCESS.poll() is None:
        print(f"🛑 關閉 {mode_cfg['launch_file']} (pgid + SIGINT)")
        try:
            pgid = os.getpgid(ROS_LAUNCH_PROCESS.pid)
            os.killpg(pgid, signal.SIGINT)
            ROS_LAUNCH_PROCESS.wait(timeout=5)
            print("✅ 子行程正常結束")
        except subprocess.TimeoutExpired:
            print("⚠️ 子行程未結束，強制 kill")
            os.killpg(pgid, signal.SIGKILL)
            ROS_LAUNCH_PROCESS.wait()

        print("♻️ 等待硬體資源釋放（相機 / GPIO / UART）...")
        await asyncio.sleep(5.0)

        ROS_LAUNCH_PROCESS = None
        print("✅ 模式已安全停止，可重新啟動")
    else:
        print(f"⚠️ 沒有正在執行的 {mode}_mode")

async def ros_mode_client():
    global MODE

    async with websockets.connect(WS_SERVER_URI) as websocket:
        print("✅ 已連線到伺服器，等待模式指令（auto / manual）...")

        while True:
            msg = await websocket.recv()
            try:
                data = json.loads(msg)
                cmd = data.get("mode_command")

                if cmd is None:
                    continue

                if cmd == "start_auto":
                    MODE = "auto"
                    await websocket.send(json.dumps({"device": "raspberry_auto"}))
                    print("🚦 切換為 AUTO 模式")
                    await launch_ros_process("auto")

                elif cmd == "start_robot":
                    MODE = "manual"
                    await websocket.send(json.dumps({"device": "raspberry_manual"}))
                    print("🚦 切換為 MANUAL 模式")
                    await launch_ros_process("manual")

                elif cmd == "stop_auto":
                    await stop_ros_process("auto")

                elif cmd == "stop_robot":
                    await stop_ros_process("manual")

                else:
                    print(f"⚠️ 不支援的指令：{cmd}")

            except Exception as e:
                print(f"❌ 錯誤：{e}")

if __name__ == "__main__":
    asyncio.run(ros_mode_client())
