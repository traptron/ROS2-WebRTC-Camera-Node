#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaRelay
from aiohttp import web
import json
import threading
import numpy as np

class VideoImageTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.img = None
        self.lock = threading.Lock()

    def recv(self):
        with self.lock:
            if self.img is not None:
                frame = self.img
            else:
                # Заглушка: черное изображение
                frame = cv2.imencode('.jpg', np.zeros((240, 240, 3), dtype=np.uint8))[1].tobytes()
        return frame

    def update_image(self, img):
        with self.lock:
            self.img = img


class WebRTCCameraNode(Node):
    def __init__(self):
        super().__init__('webrtc_camera_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.video_track = VideoImageTrack()
        self.relay = MediaRelay()
        self.pc = RTCPeerConnection()
        self.pc.addTrack(self.relay.subscribe(self.video_track))

        # HTTP сервер для сигнализации
        self.app = web.Application()
        self.app.router.add_post('/offer', self.offer_handler)
        self.runner = None

    def image_callback(self, msg):
        try:
            # Декодируем JPEG изображение
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                _, img_encoded = cv2.imencode('.jpg', cv_image)
                self.video_track.update_image(img_encoded.tobytes())
            else:
                self.get_logger().error('Failed to decode image')
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')

    async def offer_handler(self, request):
        params = await request.json()
        offer = RTCSessionDescription(sdp=params['sdp'], type=params['type'])

        await self.pc.setRemoteDescription(offer)
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)

        return web.Response(
            content_type='application/json',
            text=json.dumps({
                'sdp': self.pc.localDescription.sdp,
                'type': self.pc.localDescription.type
            })
        )

    async def start_server(self):
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        site = web.TCPSite(self.runner, '0.0.0.0', 8080)
        await site.start()
        self.get_logger().info('WebRTC signaling server started at http://0.0.0.0:8080')

    def run_asyncio_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.start_server())
        loop.run_forever()

    def start_http_server(self):
        thread = threading.Thread(target=self.run_asyncio_loop, daemon=True)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCCameraNode()
    node.start_http_server()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down WebRTC camera node...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
