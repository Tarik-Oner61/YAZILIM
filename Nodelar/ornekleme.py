#!/usr/bin/env python3
# USB İzni Vermeyi Unutma: sudo chmod 777 /dev/ttyUSB0

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import serial
import struct
import math
import time
import os

class RoverMaster(Node):
    def __init__(self):
        super().__init__('rover_master_node')
        
        # ==========================================
        # 1. AYARLAR VE SABİTLER
        # ==========================================
        self.get_logger().info("🚀 ROVER MASTER BAŞLATILIYOR (V3 - Final)...")

        # --- MOTOR AYARLARI ---
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 115200
        self.START_FRAME = 0xABCD
        self.ENABLE_MECANUM = True  # True: Yan gitme açık
        
        # Katsayılar (Kalibrasyon Şart!)
        self.SPEED_COEFF = 600.0
        self.STEER_COEFF = 300.0
        self.STRAFE_COEFF = 800.0 

        # --- YOL VE KLASÖR AYARLARI (GARANTİ YÖNTEM) ---
        user_home = os.path.expanduser("~")
        desktop_path = os.path.join(user_home, "Masaüstü") # İngilizce ise "Desktop" yap
        
        # 1. Fotoğraf Klasörü
        self.save_dir = os.path.join(desktop_path, "rover_fotograflar")
        try:
            os.makedirs(self.save_dir, exist_ok=True)
            self.get_logger().info(f'📁 Fotoğraf Klasörü: {self.save_dir}')
        except Exception as e:
            self.get_logger().error(f'❌ Klasör Hatası: {e}')

        # 2. YOLO Model Yolu (Masaüstünde arar)
        model_path = os.path.join(desktop_path, "yolov8n.pt")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'🧠 Model Yüklendi: {model_path}')
        except:
            self.get_logger().warn(f'⚠️ Model Masaüstünde bulunamadı, varsayılan indiriliyor...')
            self.model = YOLO("yolov8n.pt")

        self.CONFIDENCE_THRESHOLD = 0.6
        self.fx = 600.0
        self.cx = 320.0
        
        # --- NAVİGASYON DEĞİŞKENLERİ ---
        self.global_goal_x = 61.0
        self.global_goal_y = 34.0
        
        # Konum
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        
        # Odometri Hafızası (Önceki Hızlar)
        self.current_v = 0.0
        self.current_w = 0.0
        self.prev_v = 0.0     
        self.prev_w = 0.0
        
        self.state = "GLOBAL_APPROACH"
        self.visual_valid = False
        self.visual_x = 0.0
        self.visual_y = 0.0
        self.last_visual_time = 0.0
        
        # Sayaçlar
        self.search_start_time = None
        self.arrival_time = None
        self.last_photo_time = 0.0
        self.photo_count = 0
        self.TARGET_PHOTO_LIMIT = 5

        # Limitler
        self.DT = 0.1
        self.MAX_SPEED = 0.3
        self.MAX_TURN = 0.8

        # ==========================================
        # 2. BAĞLANTILAR (INIT)
        # ==========================================
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info("✅ Motor Kartına Bağlandı.")
        except Exception as e:
            self.get_logger().error(f"❌ MOTOR BAĞLANTISI YOK: {e}")
            self.ser = None

        self.bridge = CvBridge()
        self.rgb_frame = None
        self.depth_frame = None
        # SUB'LAR
        self.create_subscription(Image, '/realsense/rgb/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/realsense/depth/image_rect', self.depth_callback, 10)
        self.create_subscription(Point, '/mission/goal', self.mission_callback, 10)

        self.timer = self.create_timer(self.DT, self.control_loop)

    # ==========================================
    # 3. SENSÖR CALLBACK'LERİ
    # ==========================================
    def rgb_callback(self, msg):
        try:
            self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_yolo() 
        except: pass

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except: pass

    def mission_callback(self, msg):
        self.global_goal_x = msg.x
        self.global_goal_y = msg.y
        self.state = "GLOBAL_APPROACH"
        self.arrival_time = None
        self.search_start_time = None
        self.get_logger().info(f"📍 Yeni Hedef: {msg.x}, {msg.y}")

    # ==========================================
    # 4. GÖRÜNTÜ İŞLEME (YOLO)
    # ==========================================
    def process_yolo(self):
        if self.rgb_frame is None: return

        results = self.model(self.rgb_frame, verbose=False)
        target_found = False
        best_box = None
        max_conf = 0

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf > self.CONFIDENCE_THRESHOLD:
                    if conf > max_conf:
                        max_conf = conf
                        best_box = box.xyxy[0].cpu().numpy()
                        target_found = True
        
        if target_found and self.depth_frame is not None:
            x1, y1, x2, y2 = map(int, best_box)
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            try:
                dist_mm = self.depth_frame[center_y, center_x]
                if dist_mm > 0:
                    dist_m = dist_mm / 1000.0
                    self.visual_x = float(dist_m)
                    self.visual_y = float(-1 * (center_x - self.cx) * dist_m / self.fx)
                    self.visual_valid = True
                    self.last_visual_time = time.time()
                    cv2.rectangle(self.rgb_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            except: pass
        
        cv2.imshow("Rover View", self.rgb_frame)
        cv2.waitKey(1)

    # ==========================================
    # 5. NAVİGASYON VE KONTROL (BEYİN)
    # ==========================================
    def control_loop(self):
        # ---------------------------------------------------------
        # A) ODOMETRİ (KONUM TAHMİNİ) - DÜZELTİLDİ
        # ---------------------------------------------------------
        # Trapez Yöntemi: (Önceki Hız + Şu Anki Hız) / 2
        avg_v = (self.current_v + self.prev_v) / 2.0
        avg_w = (self.current_w + self.prev_w) / 2.0
        
        delta_yaw = avg_w * self.DT
        self.pose_yaw += delta_yaw
        
        self.pose_x += avg_v * math.cos(self.pose_yaw) * self.DT
        self.pose_y += avg_v * math.sin(self.pose_yaw) * self.DT
        # ---------------------------------------------------------

        current_time = time.time()
        if current_time - self.last_visual_time > 1.0:
            self.visual_valid = False

        # Karar verilecek hızlar
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        # --- DURUM MAKİNESİ ---
        if self.state == "STOPPED":
            if self.arrival_time is None:
                self.arrival_time = current_time
                self.last_photo_time = current_time
                self.get_logger().info("📸 Hedefteyim! Fotoğraf modu başlatılıyor...")
            elif current_time - self.arrival_time > 3.0:
                 if self.state != "MISSION_COMPLETE":
                    if current_time - self.last_photo_time > 1.0:
                        self.take_snapshot()
                        self.photo_count += 1
                        self.last_photo_time = current_time
                        if self.photo_count >= self.TARGET_PHOTO_LIMIT:
                            self.state = "MISSION_COMPLETE"
                            self.get_logger().info("🏆 GÖREV BİTTİ!")

        elif self.state == "MISSION_COMPLETE":
            pass 

        elif self.visual_valid:
            if self.state in ["GLOBAL_APPROACH", "SEARCHING"]:
                self.get_logger().info("👁️ Taş Görüldü! Yaklaşılıyor...")
            self.state = "VISUAL_SERVOING"
            
            if self.visual_x < 0.5:
                self.state = "STOPPED"
            else:
                linear_x = self.MAX_SPEED
                if abs(self.visual_y) > 0.2: linear_x = 0.1 
                linear_y = self.visual_y * 0.8 
                angular_z = 0.0 

        else:
            dx = self.global_goal_x - self.pose_x
            dy = self.global_goal_y - self.pose_y
            dist_to_goal = math.sqrt(dx**2 + dy**2)

            if dist_to_goal < 0.5:
                self.state = "SEARCHING"
                if self.search_start_time is None:
                    self.search_start_time = current_time
                    self.get_logger().info("🌀 Hedefteyim ama taş yok. SARMAL ARAMA!")
                elapsed = current_time - self.search_start_time
                if elapsed < 5.0:
                    angular_z = 0.6
                else:
                    speed_boost = (elapsed - 5.0) * 0.01
                    linear_x = min(0.1 + speed_boost, 0.25)
                    angular_z = 0.6
                    if elapsed > 20.0:
                        self.state = "MISSION_COMPLETE"
                        self.get_logger().warn("❌ Bulunamadı.")
            else:
                self.state = "GLOBAL_APPROACH"
                self.search_start_time = None
                target_angle = math.atan2(dy, dx)
                err_yaw = target_angle - self.pose_yaw
                while err_yaw > math.pi: err_yaw -= 2*math.pi
                while err_yaw < -math.pi: err_yaw += 2*math.pi
                
                if abs(err_yaw) > 0.3:
                    angular_z = 0.8 if err_yaw > 0 else -0.8
                else:
                    linear_x = self.MAX_SPEED
                    angular_z = err_yaw * 1.0

        # Limitler
        linear_x = max(min(linear_x, self.MAX_SPEED), -self.MAX_SPEED)
        linear_y = max(min(linear_y, self.MAX_SPEED), -self.MAX_SPEED)
        angular_z = max(min(angular_z, self.MAX_TURN), -self.MAX_TURN)

        # ---------------------------------------------------------
        # B) GÜNCELLEME SIRASI (ÇOK ÖNEMLİ) - DÜZELTİLDİ
        # ---------------------------------------------------------
        # Önce: Şimdiki hızı 'Eski' olarak sakla
        self.prev_v = self.current_v
        self.prev_w = self.current_w
        
        # Sonra: Yeni hızı 'Şimdiki' olarak kaydet
        self.current_v = linear_x
        self.current_w = angular_z
        
        # En Son: Motora gönder
        self.send_to_motor(linear_x, linear_y, angular_z)
        # ---------------------------------------------------------

    # ==========================================
    # 6. MOTOR SÜRÜCÜSÜ
    # ==========================================
    def send_to_motor(self, v_x, v_y, v_z):
        if self.ser is None: return

        speed = int(v_x * self.SPEED_COEFF)
        steer = int(v_z * self.STEER_COEFF)
        strafe = int(v_y * self.STRAFE_COEFF)

        speed = max(min(speed, 1000), -1000)
        steer = max(min(steer, 1000), -1000)
        strafe = max(min(strafe, 1000), -1000)

        try:
            packet = None
            if self.ENABLE_MECANUM:
                checksum = self.START_FRAME ^ steer ^ speed ^ strafe
                packet = struct.pack('<HhhhH', self.START_FRAME, steer, speed, strafe, checksum)
            else:
                checksum = self.START_FRAME ^ steer ^ speed
                packet = struct.pack('<HhhH', self.START_FRAME, steer, speed, checksum)
            
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"Motor Hatası: {e}")

    # ==========================================
    # 7. KAMERA KAYIT (FOTOĞRAF)
    # ==========================================
    def take_snapshot(self):
        if self.rgb_frame is not None:
            timestamp = time.strftime("%M%S") 
            filename = f"sample_{self.photo_count + 1}_{timestamp}.jpg"
            foto_path = os.path.join(self.save_dir, filename)
            
            success = cv2.imwrite(foto_path, self.rgb_frame)
            if success:
                self.get_logger().info(f"💾 KAYDEDİLDİ: {filename}")
            else:
                self.get_logger().error("❌ Kayıt Başarısız!")

def main(args=None):
    rclpy.init(args=args)
    node = RoverMaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
