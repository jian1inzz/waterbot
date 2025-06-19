#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool
from cv_bridge import CvBridge

import asyncio, websockets, base64, cv2, threading, json

class CameraWebSocketNode(Node):
    def __init__(self):
        super().__init__('camera_ws_node')

        # -------- 先強制釋放相機（如果被其他殘留程式佔用） --------
        try:
            temp_cap = cv2.VideoCapture(0)
            if temp_cap.isOpened():
                temp_cap.release()
                self.get_logger().info("♻️ 檢查：已釋放殘留相機資源")
        except Exception as e:
            self.get_logger().warn(f"⚠️ 檢查相機資源失敗：{e}")

        # -------- ROS publishers --------
        self.img_pub  = self.create_publisher(Image,   '/image_raw', 10)
        self.cmd_pub  = self.create_publisher(String,  '/yolo_cmd',  10)
        self.ang_pub  = self.create_publisher(Float32, '/yolo_angle', 10)
        self.ws_status_pub = self.create_publisher(Bool, '/websocket_connected', 1)

        self.bridge = CvBridge()
        self.prev_cmd   = None
        self.prev_angle = None

        # -------- 電量狀態 --------
        self.battery_percent = None
        self.prev_battery_percent = None
        self.create_subscription(Float32, '/battery_percent', self.cb_battery, 10)

        # -------- 打開相機 --------
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ 相機開啟失敗")
            return
        self.get_logger().info("✅ 相機成功打開")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.get_logger().info(
            f"✅ 相機成功打開 → {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}×"
            f"{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}"
        )

        # 10 Hz 定時拍照 + 發影像 + WebSocket 傳輸
        self.create_timer(0.06, self.timer_cb)

        # -------- WebSocket --------
        self.ws_uri = "ws://172.20.10.2:8765"
        self.ws   = None
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        asyncio.run_coroutine_threadsafe(self.connect_ws(), self.loop)

        # -------- 加入清理函式 --------
        rclpy.get_default_context().on_shutdown(self.cleanup)

    # ================== Timer ==================
    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("⚠️ 無法讀取影像")
            return

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        if self.ws:
            asyncio.run_coroutine_threadsafe(self.ws_send(frame), self.loop)

    # ================== 電量回呼 ==================
    def cb_battery(self, msg: Float32):
        self.battery_percent = int(msg.data)  # 不要小數點

        # 如果與上次不同才傳送
        if self.battery_percent != self.prev_battery_percent and self.ws:
            try:
                battery_json = json.dumps({
                    "battery_percent": self.battery_percent
                })
                asyncio.run_coroutine_threadsafe(self.ws.send(battery_json), self.loop)
                self.prev_battery_percent = self.battery_percent
            except Exception as e:
                self.get_logger().warning(f"⚠️ 傳送電量失敗: {e}")
                self.ws = None

    # ================== WebSocket 連線 ==================
    async def connect_ws(self):
        try:
            self.ws = await websockets.connect(self.ws_uri)
            self.get_logger().info("✅ 已連上 WebSocket Server")
            self.ws_status_pub.publish(Bool(data=True))
            asyncio.ensure_future(self.ws_receive())
        except Exception as e:
            self.get_logger().error(f"❌ WebSocket 連線失敗: {e}")
            self.ws = None

    async def ws_send(self, frame):
        try:
            _, buf = cv2.imencode('.jpg', frame)
            await self.ws.send(buf.tobytes())
        except Exception as e:
            self.get_logger().warning(f"⚠️ 傳送影像失敗: {e}")
            self.ws = None

    async def ws_receive(self):
        try:
            async for message in self.ws:
                try:
                    data = json.loads(message)
                    cmd   = data.get("cmd")
                    angle = data.get("angle_deg")
                except json.JSONDecodeError:
                    cmd   = message.strip().lower()
                    angle = None

                if cmd is None:
                    continue

                if cmd in ("left", "left_fast", "center",
                           "right", "right_fast", "forward", "stop"):
                    str_msg = String(); str_msg.data = cmd
                    self.cmd_pub.publish(str_msg)

                    if angle is not None:
                        ang_val = float(angle)
                        ang_msg = Float32(); ang_msg.data = ang_val
                        self.ang_pub.publish(ang_msg)

                        
                else:
                    self.get_logger().warning(f"⚠️ 未知指令: {message}")
        except Exception as e:
            self.get_logger().error(f"❌ 接收失敗: {e}")
            self.ws = None

    # ================== 清理函式 ==================
    def cleanup(self):
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("📷 相機已釋放")
        if self.ws:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.get_logger().info("🔌 WebSocket 已關閉事件迴圈")

def main(args=None):
    rclpy.init(args=args)
    node = CameraWebSocketNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
