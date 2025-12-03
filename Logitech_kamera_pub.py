#!/usr/bin/env python3
"""
Logitech C270 USB Kamera Publisher Node
Tüm kamera stream'lerini ROS topic'leri olarak yayınlar
Logitech kameradan alınan kamera görüntülerini ROS sistem mimarisine yayınlar
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class LogitechPublisher(Node):
    """
    Logitech C270 USB kamerasından tüm stream'leri ROS 2'ye yayınlar
    
    Yayınlanan Topic'ler:
    - /logitech/image_raw           : RGB görüntü (640x480 veya 1280x720)
    - /logitech/image_compressed    : JPEG sıkıştırılmış görüntü
    - /logitech/camera_info         : Kamera kalibrasyon bilgisi
    - /logitech/image_mono          : Gri tonlama görüntü
    - /logitech/image_rect          : Düzeltilmiş (rectified) görüntü
    """
    
    def __init__(self):
        super().__init__('logitech_publisher_node')
        
        # Parametreler
        self.declare_parameter('camera_id', 0)  # /dev/video0
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('enable_compressed', True)
        self.declare_parameter('enable_mono', True)
        self.declare_parameter('brightness', 128)  # 0-255
        self.declare_parameter('contrast', 32)     # 0-255
        self.declare_parameter('saturation', 32)   # 0-255
        
        self.camera_id = self.get_parameter('camera_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.auto_exposure = self.get_parameter('auto_exposure').value
        self.enable_compressed = self.get_parameter('enable_compressed').value
        self.enable_mono = self.get_parameter('enable_mono').value
        self.brightness = self.get_parameter('brightness').value
        self.contrast = self.get_parameter('contrast').value
        self.saturation = self.get_parameter('saturation').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # VideoCapture objesi
        self.cap = None
        
        # Publishers oluştur
        self.create_publishers()
        
        # Kamerayı başlat
        self.initialize_camera()
        
        # Timer ile frame'leri publish et
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        
        # İstatistikler
        self.frame_count = 0
        self.dropped_frames = 0
        self.start_time = time.time()
        
        # Camera info (basit kalibrasyon - gerçek kalibrasyonla değiştirilmeli)
        self.camera_info = self.create_default_camera_info()
        
        self.get_logger().info('🎥 Logitech Publisher Node başlatıldı')
        self.get_logger().info(f'📸 Çözünürlük: {self.width}x{self.height} @ {self.fps} FPS')
        self.get_logger().info(f'🔧 JPEG Kalitesi: {self.jpeg_quality}')
        self.get_logger().info(f'🔧 Auto Exposure: {"Aktif" if self.auto_exposure else "Pasif"}')
        self.get_logger().info(f'🔧 Compressed Image: {"Aktif" if self.enable_compressed else "Pasif"}')
        self.get_logger().info(f'🔧 Mono Image: {"Aktif" if self.enable_mono else "Pasif"}')
    
    def create_publishers(self):
        """Tüm ROS publisher'ları oluştur"""
        
        # Ana RGB görüntü
        self.pub_image_raw = self.create_publisher(
            Image, '/logitech/image_raw', 10)
        
        # Compressed (JPEG) görüntü
        if self.enable_compressed:
            self.pub_image_compressed = self.create_publisher(
                CompressedImage, '/logitech/image_compressed', 10)
        
        # Mono (gri tonlama) görüntü
        if self.enable_mono:
            self.pub_image_mono = self.create_publisher(
                Image, '/logitech/image_mono', 10)
        
        # Rectified görüntü (düzeltilmiş - kalibrasyon sonrası)
        self.pub_image_rect = self.create_publisher(
            Image, '/logitech/image_rect', 10)
        
        # Camera info
        self.pub_camera_info = self.create_publisher(
            CameraInfo, '/logitech/camera_info', 10)
        
        self.get_logger().info('✅ Tüm publisher\'lar oluşturuldu')
    
    def initialize_camera(self):
        """Logitech kamerasını başlat ve yapılandır"""
        try:
            # VideoCapture ile kamerayı aç
            self.cap = cv2.VideoCapture(self.camera_id)
            
            if not self.cap.isOpened():
                raise RuntimeError(f'Kamera açılamadı: /dev/video{self.camera_id}')
            
            # Çözünürlük ayarla
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # MJPEG codec kullan (daha iyi performans)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            
            # Kamera ayarları
            if self.auto_exposure:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # Auto mode
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Manual mode
            
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
            self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
            self.cap.set(cv2.CAP_PROP_SATURATION, self.saturation)
            
            # Gerçek çözünürlüğü kontrol et
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            
            self.get_logger().info(f'✅ Kamera başlatıldı: {actual_width}x{actual_height} @ {actual_fps} FPS')
            
            # İlk birkaç frame'i atla (kamera stabilize olsun)
            for _ in range(10):
                self.cap.read()
            
        except Exception as e:
            self.get_logger().error(f'❌ Kamera başlatma hatası: {e}')
            raise
    
    def publish_frame(self):
        """Her timer tick'inde frame al ve publish et"""
        try:
            # Frame oku
            ret, frame = self.cap.read()
            
            if not ret:
                self.dropped_frames += 1
                if self.dropped_frames % 10 == 0:
                    self.get_logger().warn(f'⚠️ Frame okunamadı! Toplam kayıp: {self.dropped_frames}')
                return
            
            # Timestamp oluştur
            timestamp = self.get_clock().now().to_msg()
            
            # 1. Raw RGB Image
            self.publish_raw_image(frame, timestamp)
            
            # 2. Compressed (JPEG) Image
            if self.enable_compressed:
                self.publish_compressed_image(frame, timestamp)
            
            # 3. Mono (Grayscale) Image
            if self.enable_mono:
                self.publish_mono_image(frame, timestamp)
            
            # 4. Rectified Image (şimdilik aynı, kalibrasyon yapılınca değişir)
            self.publish_rectified_image(frame, timestamp)
            
            # 5. Camera Info
            self.camera_info.header.stamp = timestamp
            self.pub_camera_info.publish(self.camera_info)
            
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'❌ Frame publish hatası: {e}')
    
    def publish_raw_image(self, frame, timestamp):
        """Raw RGB görüntüsünü publish et"""
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'logitech_optical_frame'
        
        self.pub_image_raw.publish(msg)
    
    def publish_compressed_image(self, frame, timestamp):
        """JPEG sıkıştırılmış görüntüyü publish et"""
        # JPEG'e sıkıştır
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        _, jpeg_data = cv2.imencode('.jpg', frame, encode_param)
        
        # CompressedImage mesajı oluştur
        msg = CompressedImage()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'logitech_optical_frame'
        msg.format = 'jpeg'
        msg.data = jpeg_data.tobytes()
        
        self.pub_image_compressed.publish(msg)
    
    def publish_mono_image(self, frame, timestamp):
        """Gri tonlama görüntüyü publish et"""
        # BGR'den grayscale'e çevir
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'logitech_optical_frame'
        
        self.pub_image_mono.publish(msg)
    
    def publish_rectified_image(self, frame, timestamp):
        """Düzeltilmiş (rectified) görüntüyü publish et"""
        # Gerçek kalibrasyon matrisleri olsaydı undistort yapardık:
        # rectified = cv2.undistort(frame, K, D, None, new_K)
        
        # Şimdilik aynı görüntüyü publish ediyoruz
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'logitech_optical_frame'
        
        self.pub_image_rect.publish(msg)
    
    def create_default_camera_info(self):
        """Varsayılan camera info oluştur (gerçek kalibrasyonla değiştirilmeli)"""
        msg = CameraInfo()
        msg.header.frame_id = 'logitech_optical_frame'
        
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = 'plumb_bob'
        
        # Distortion coefficients - tahmini değerler (kalibrasyon yapılmalı!)
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Intrinsic camera matrix (K) - tahmini değerler
        # Bu değerler kamera kalibrasyon tool'u ile elde edilmeli
        fx = self.width * 1.0  # focal length x (piksel cinsinden)
        fy = self.height * 1.0  # focal length y (piksel cinsinden)
        cx = self.width / 2.0   # principal point x
        cy = self.height / 2.0  # principal point y
        
        msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]
        
        # Rectification matrix (R) - identity
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix (P)
        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return msg
    
    def print_stats(self):
        """İstatistikleri yazdır"""
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        self.get_logger().info('📊 Performans İstatistikleri:')
        self.get_logger().info(f'  Toplam Frame: {self.frame_count}')
        self.get_logger().info(f'  Kayıp Frame: {self.dropped_frames}')
        self.get_logger().info(f'  Ortalama FPS: {fps:.2f}')
        self.get_logger().info(f'  Çalışma Süresi: {elapsed:.1f} saniye')
    
    def destroy_node(self):
        """Node kapatılırken temizlik yap"""
        self.get_logger().info('🛑 Logitech Publisher Node kapatılıyor...')
        self.print_stats()
        
        if self.cap:
            self.cap.release()
            self.get_logger().info('✅ Kamera serbest bırakıldı')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = LogitechPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
