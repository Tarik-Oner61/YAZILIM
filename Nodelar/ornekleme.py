#!/usr/bin/env python3
# GÜVENLİK NOTU: TTY İzni için şu komutu bir kez terminale yazın (Bilgisayarı yeniden başlatın):
# sudo usermod -a -G dialout $USER

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
from threading import Lock  

class RoverMaster(Node):
    def __init__(self):
        super().__init__('rover_master_node')
        
        # ==========================================
        # 1. AYARLAR VE GÜVENLİK
        # ==========================================
        self.get_logger().info("🛡️ ROVER MASTER V5 (ROBUST) BAŞLATILIYOR...")
        
        # THREAD KİLİDİ (DeepSeek Uyarısı #2)
        # Kamera verisi yazılırken, navigasyon okumaya çalışırsa çökmesin diye kapıyı kilitliyoruz.
        self.data_lock = Lock()

        # --- MOTOR BAĞLANTISI ---
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 115200
        self.START_FRAME = 0xABCD
        self.ENABLE_MECANUM = True
        
        # KATSAYILAR (Parametre sunucusu yerine basitlik için burada)
        self.SPEED_COEFF = 600.0
        self.STEER_COEFF = 300.0
        self.STRAFE_COEFF = 800.0 

        # --- AKILLI KLASÖR BULUCU (DeepSeek Uyarısı #7) ---
        user_home = os.path.expanduser("~")
        possible_paths = ["Masaüstü", "Desktop", "Desktop"] # Öncelik sırası
        self.desktop_path = user_home # Bulamazsa Home'a kaydeder
        
        for p in possible_paths:
            check_path = os.path.join(user_home, p)
            if os.path.exists(check_path):
                self.desktop_path = check_path
                break
        
        self.save_dir = os.path.join(self.desktop_path, "rover_fotograflar")
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f'📁 Kayıt Yeri: {self.save_dir}')

        # --- MODEL YÜKLEME (DeepSeek Uyarısı #5) ---
        model_name = "yolov8n.pt"
        model_path = os.path.join(self.desktop_path, model_name)
        try:
            # Önce masaüstüne bak, yoksa varsayılan yere bak
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
                self.get_logger().info(f'🧠 Yerel Model Yüklendi: {model_path}')
            else:
                self.get_logger().warn(f'⚠️ Model Masaüstünde Yok! İndirilmeye çalışılıyor...')
                self.model = YOLO(model_name)
        except Exception as e:
            self.get_logger().error(f'❌ KRİTİK HATA: YOLO Modeli Yüklenemedi! İnternet veya Dosya Yok. Hata: {e}')
            # Robotu durdurmak yerine exception fırlatıp programı kapatıyoruz ki hatayı gör.
            raise e

        # --- NAVİGASYON DEĞİŞKENLERİ ---
        self.global_goal_x = 61.0
        self.global_goal_y = 34.0
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        
        # Odometri & Hız
        self.current_v = 0.0
        self.current_w = 0.0
        self.prev_v = 0.0     
        self.prev_w = 0.0
        
        # State Machine
        self.state = "GLOBAL_APPROACH"
        self.visual_valid = False
        self.visual_x = 0.0
        self.visual_y = 0.0
        self.last_visual_time = 0.0
        
        # Zamanlayıcılar 
        self.last_loop_time = self.get_clock().now()
        self.search_start_time = None
        self.arrival_time = None
        self.last_photo_time = 0.0
        self.photo_count = 0
        self.TARGET_PHOTO_LIMIT = 5

        # Performans
        self.last_yolo_time = 0
        self.YOLO_INTERVAL = 0.15 # ~7 FPS sınırı

        # ==========================================
        # 2. BAĞLANTILAR
        # ==========================================
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info("✅ Seri Port Bağlandı.")
        except Exception as e:
            self.get_logger().error(f"❌ SERİ PORT HATASI: {e}")
            self.ser = None

        self.bridge = CvBridge()
        self.rgb_frame = None
        self.depth_frame = None
        
        self.create_subscription(Image, '/realsense/rgb/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/realsense/depth/image_rect', self.depth_callback, 10)
        self.create_subscription(Point, '/mission/goal', self.mission_callback, 10)

        # Kontrol döngüsü 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    # ==========================================
    # 3. THREAD-SAFE CALLBACK'LER
    # ==========================================
    def rgb_callback(self, msg):
        # Kilit mekanizması: Ben yazarken kimse okumasın
        with self.data_lock:
            try:
                self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except: pass
        
        # YOLO'yu ana thread'i boğmamak için burada asenkron gibi çalıştırıyoruz (Frame Skip)
        now = time.time()
        if now - self.last_yolo_time > self.YOLO_INTERVAL:
            self.process_yolo() 
            self.last_yolo_time = now

    def depth_callback(self, msg):
        with self.data_lock:
            try:
                self.depth_frame = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            except: pass

    def mission_callback(self, msg):
        self.global_goal_x = msg.x
        self.global_goal_y = msg.y
        self.state = "GLOBAL_APPROACH"
        self.photo_count = 0
        self.arrival_time = None
        self.search_start_time = None
        self.get_logger().info(f"📍 Yeni Görev Alındı: {msg.x}, {msg.y}")

    # ==========================================
    # 4. GÖRÜNTÜ İŞLEME
    # ==========================================
    def process_yolo(self):
        # İşlem yapacağımız frame'in kopyasını alıyoruz (Thread Safety)
        current_rgb = None
        current_depth = None
        
        with self.data_lock:
            if self.rgb_frame is None: return
            current_rgb = self.rgb_frame.copy()
            if self.depth_frame is not None:
                current_depth = self.depth_frame.copy()

        # DeepSeek Uyarısı #6: waitKey ROS thread'ini bloklar.
        # Bunu headless sistemlerde (ekransız) çalıştırmak için korumaya alıyoruz.
        has_display = os.environ.get("DISPLAY") is not None

        results = self.model(current_rgb, verbose=False)
        target_found = False
        best_box = None
        max_conf = 0

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf > 0.6: # Eşik Değeri
                    if conf > max_conf:
                        max_conf = conf
                        best_box = box.xyxy[0].cpu().numpy()
                        target_found = True
        
        if target_found and current_depth is not None:
            # Boyut kontrolü (DeepSeek Uyarısı #13)
            if current_depth.shape == current_rgb.shape[:2]:
                x1, y1, x2, y2 = map(int, best_box)
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                try:
                    dist_mm = current_depth[center_y, center_x]
                    if dist_mm > 0:
                        dist_m = dist_mm / 1000.0
                        
                        # Thread-Safe Yazma
                        with self.data_lock:
                            self.visual_x = float(dist_m)
                            self.visual_y = float(-1 * (center_x - 320.0) * dist_m / 600.0)
                            self.visual_valid = True
                            self.last_visual_time = time.time()
                        
                        if has_display:
                            cv2.rectangle(current_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                except Exception as e: 
                    pass # Tekil piksel okuma hatası önemsiz

        if has_display:
            cv2.imshow("Rover View", current_rgb)
            cv2.waitKey(1)

    # ==========================================
    # 5. KONTROL DÖNGÜSÜ (BEYİN)
    # ==========================================
    def control_loop(self):
        # 1. GERÇEK ZAMAN FARKI (DT) HESABI (DeepSeek Uyarısı #12)
        now = self.get_clock().now()
        dt_nano = (now - self.last_loop_time).nanoseconds
        dt = dt_nano / 1e9  # Saniyeye çevir
        self.last_loop_time = now
        
        if dt > 1.0: dt = 0.1 # İlk açılışta saçmalamasın diye koruma

        # 2. ODOMETRİ (Trapez Yöntemi)
        # DeepSeek Uyarısı #3: Burada current_v bir önceki döngüden kalan değerdir.
        # Bu yüzden (Eski + Daha Eski) / 2 yapmış oluyoruz. 
        # En doğrusu: Önce hesapla, sonra güncelle.
        
        avg_v = (self.current_v + self.prev_v) / 2.0
        avg_w = (self.current_w + self.prev_w) / 2.0
        
        # Açı hesabı
        self.pose_yaw += avg_w * dt
        # Normalize et (-pi, +pi)
        self.pose_yaw = math.atan2(math.sin(self.pose_yaw), math.cos(self.pose_yaw))
        
        # Konum hesabı
        self.pose_x += avg_v * math.cos(self.pose_yaw) * dt
        self.pose_y += avg_v * math.sin(self.pose_yaw) * dt

        # 3. STATE KONTROLÜ VE KARAR
        current_sys_time = time.time()
        
        # Visual Hysteresis (Görsel Hafıza)
        # DeepSeek Uyarısı #4: Görsel kaybolursa ne olacak?
        with self.data_lock:
            # 2 saniye görmezsen "Kaybettim" de
            if current_sys_time - self.last_visual_time > 2.0:
                self.visual_valid = False

        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        if self.state == "STOPPED":
            if self.arrival_time is None:
                self.arrival_time = current_sys_time
                self.last_photo_time = current_sys_time
                self.get_logger().info("📸 Fotoğraf Modu...")
            
            elif current_sys_time - self.arrival_time > 3.0: 
                if self.state != "MISSION_COMPLETE":
                    if current_sys_time - self.last_photo_time > 1.0:
                        self.take_snapshot()
                        self.photo_count += 1
                        self.last_photo_time = current_sys_time
                        if self.photo_count >= self.TARGET_PHOTO_LIMIT:
                            self.state = "MISSION_COMPLETE"
                            self.get_logger().info("🏆 GÖREV TAMAMLANDI!")

        elif self.state == "MISSION_COMPLETE":
            # Durma garantisi
            linear_x = 0.0
            linear_y = 0.0
            angular_z = 0.0

        elif self.visual_valid:
            self.state = "VISUAL_SERVOING"
            # Thread-safe okuma
            vx_target, vy_target = 0, 0
            with self.data_lock:
                vx_target = self.visual_x
                vy_target = self.visual_y
            
            if vx_target < 0.5:
                self.state = "STOPPED"
            else:
                linear_x = self.MAX_SPEED
                if abs(vy_target) > 0.2: linear_x = 0.1
                linear_y = vy_target * 1.0 
                angular_z = vy_target * 0.5 
        
        else:
            # Görsel temas yoksa
            # DeepSeek Uyarısı #4 Düzeltmesi: Eğer VISUAL_SERVOING modundayken görüntü gittiyse,
            # SEARCHING moduna geri dönmesini sağlıyoruz.
            if self.state == "VISUAL_SERVOING":
                self.state = "SEARCHING"
                self.search_start_time = current_sys_time
                self.get_logger().warn("⚠️ Görsel Temas Kayboldu! Tekrar aranıyor...")

            dx = self.global_goal_x - self.pose_x
            dy = self.global_goal_y - self.pose_y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 0.5:
                self.state = "SEARCHING"
                if self.search_start_time is None:
                    self.search_start_time = current_sys_time
                    self.get_logger().info("🌀 Hedef Bölgesi. Sarmal Arama...")
                
                elapsed = current_sys_time - self.search_start_time
                if elapsed < 5.0:
                    angular_z = 0.6
                elif elapsed < 25.0:
                    # Sarmal
                    boost = (elapsed - 5.0) * 0.01
                    linear_x = min(0.1 + boost, 0.25)
                    angular_z = 0.6
                else:
                    self.state = "MISSION_COMPLETE"
                    self.get_logger().error("❌ Taş Bulunamadı.")
            else:
                self.state = "GLOBAL_APPROACH"
                self.search_start_time = None
                t_angle = math.atan2(dy, dx)
                err = t_angle - self.pose_yaw
                # Açı normalizasyonu
                err = math.atan2(math.sin(err), math.cos(err))
                
                if abs(err) > 0.3:
                    angular_z = 0.8 if err > 0 else -0.8
                else:
                    linear_x = self.MAX_SPEED
                    angular_z = err * 1.0

        # Limitler
        linear_x = max(min(linear_x, 0.3), -0.3)
        linear_y = max(min(linear_y, 0.3), -0.3)
        angular_z = max(min(angular_z, 0.8), -0.8)

        # Değerleri sakla (Bir sonraki tur için)
        self.prev_v = self.current_v
        self.prev_w = self.current_w
        self.current_v = linear_x
        self.current_w = angular_z

        self.send_to_motor(linear_x, linear_y, angular_z)

    # ==========================================
    # 6. GÜVENLİ MOTOR SÜRÜCÜSÜ
    # ==========================================
    def send_to_motor(self, v_x, v_y, v_z):
        if self.ser is None: return

        speed = int(v_x * self.SPEED_COEFF)
        steer = int(v_z * self.STEER_COEFF)
        strafe = int(v_y * self.STRAFE_COEFF)

        # Limitler (-1000, 1000)
        speed = max(min(speed, 1000), -1000)
        steer = max(min(steer, 1000), -1000)
        strafe = max(min(strafe, 1000), -1000)

        try:
            # DeepSeek Uyarısı: Negatif Checksum Çökmesi (0xFFFF Maskeleme ile çözüldü)
            if self.ENABLE_MECANUM:
                checksum = (self.START_FRAME ^ steer ^ speed ^ strafe) & 0xFFFF
                packet = struct.pack('<HhhhH', self.START_FRAME, steer, speed, strafe, checksum)
            else:
                checksum = (self.START_FRAME ^ steer ^ speed) & 0xFFFF
                packet = struct.pack('<HhhH', self.START_FRAME, steer, speed, checksum)
            
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"Motor Yazma Hatası: {e}")

    def take_snapshot(self):
        # Kopyasını alıp kaydedelim ki o sırada frame değişmesin
        frame_to_save = None
        with self.data_lock:
            if self.rgb_frame is not None:
                frame_to_save = self.rgb_frame.copy()
        
        if frame_to_save is not None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            fname = f"sample_{self.photo_count + 1}_{ts}.jpg"
            path = os.path.join(self.save_dir, fname)
            if cv2.imwrite(path, frame_to_save):
                self.get_logger().info(f"💾 Kayıt: {fname}")
            else:
                self.get_logger().error("❌ Kayıt Başarısız")

def main(args=None):
    rclpy.init(args=args)
    # DeepSeek Uyarısı #8: Exception Handling
    try:
        node = RoverMaster()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"KRİTİK HATA: {e}")
    finally:
        # Kapatırken motorları durdurabiliriz (Opsiyonel)
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
