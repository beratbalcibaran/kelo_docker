import cv2
from sensor_msgs.msg import CompressedImage, Image

JPEG_QUALITY = 60


class ViewPublisher:
    def __init__(self, node, topic, depth=2, quality=JPEG_QUALITY):
        self.raw = node.create_publisher(Image, topic, depth)
        self.jpeg = node.create_publisher(CompressedImage, topic + '/compressed', depth)
        self.quality = int(quality)

    def get_subscription_count(self):
        return self.raw.get_subscription_count() + self.jpeg.get_subscription_count()

    def publish(self, header, rgb):
        if self.raw.get_subscription_count() > 0:
            msg = Image()
            msg.header = header
            msg.height, msg.width = rgb.shape[:2]
            msg.encoding, msg.is_bigendian, msg.step = 'rgb8', 0, rgb.shape[1] * 3
            msg.data = rgb.tobytes()
            self.raw.publish(msg)
        if self.jpeg.get_subscription_count() > 0:
            ok, buf = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                msg = CompressedImage()
                msg.header = header
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
                self.jpeg.publish(msg)
