# 🚀 Rover Yazılım Geliştirme Deposu

Bu depo, ERC Rover projesi kapsamında geliştirilen tüm yazılım bileşenlerini içerir. Depo, ekip üyelerinin görevlerini takip edebilmesi, kodlarını paylaşabilmesi ve ortak bir geliştirme ortamında çalışabilmesi için oluşturulmuştur.

## 📌 Proje İçeriği

Bu projede geliştirilen ana modüller:

* Gazebo Simülasyonu ve Ortam Oluşturma
* Navigasyon Görevi
* Astrobiyoloji Görevi
* Örnekleme Görevi
* Arayüz
* Sondaj Görevi
* Roverın Haritalandırması İçin Çözümler
* Manuel Araç Kontrol

Her modül ilgili ekip üyesi tarafından geliştirilir ve düzenli olarak bu depoya yüklenir.

---

## 🧠 Sistem Mimarisi

Aşağıdaki mimari diyagram, Rover yazılım ve donanım bileşenleri arasındaki veri akışını ve haberleşme yapısını göstermektedir.

### Genel Yapı

* **Yer İstasyonu** ile **Rover**, modem IP adresleri üzerinden çift yönlü haberleşir.
* Rover üzerinde çalışan **ROS tabanlı node’lar**, sensör verilerini ve motor kontrol komutlarını yönetir.
* **RealSense**, **Logitech** kamera ve **telemetri** verileri, ROS publisher node’ları üzerinden yayınlanır; görevler için yazılan node’lar bu kamera ve sensör publisher’larına subscriber olarak abone olur.
* **ZMQ_Bridge_Node kodu**, ROS ile görev için yazılan node’lar arasında publisher–subscriber bağlantısı oluşturur.
* **ZMQ_Bridge_Node kodu**, yer istasyonu arayüzü ile rover arasında ağ üzerinden veri köprüsü görevi görür.
* Motor kontrolü, **UART (USB‑TTL)** üzerinden BLDC motor anakartlarına iletilir.

### Veri ve Kontrol Akışı

1. Sensör verileri ROS publisher’lar tarafından yayınlanır.
2. Ana ROS node’ları bu verileri işleyerek karar üretir.
3. Kararlar hem arayüze (ZMQ + IP) gönderilir hem de motor kontrol birimlerine UART üzerinden iletilir.
4. Yer istasyonundan gelen manuel veya görev tabanlı komutlar aynı hat üzerinden Rover’a ulaşır.

### Mimari Diyagram

<img width="762" height="642" alt="Sistem_mimarisi drawio" src="https://github.com/user-attachments/assets/61351730-bfee-41a5-98b6-fb131d07decc" />


---

## 👥 Ekip Görev Dağılımı (Renk Kodlu)

🟥 **Saadettin** — Gazebo ve Simülasyon Geliştirme
---------------------------------------------------
- Gazebo’da simülasyon ortamı oluşturma
- CAD dosyalarını Gazebo’ya ekleme
- ROS kodunun Gazebo ortamında test edilmesi
- Sensörlerden gelen verilerin simülasyonda modellenmesi

🟦 **Ali** — Navigasyon & AR‑GPS İşleme
----------------------------------------
- ARuco tespiti
- GPS konumlandırma
- En uygun rota planlama algoritması geliştirme
- otonom sürüş kontrol

🟩 **Defne** — Sondaj Görevi
----------------------------
- Prob tespiti
- Otonom sürüş kontrol

🟪 **Mustafa** — Örnekleme Görevi
----------------------------------
- Kaya veya taş tespiti
- Nesne tespit edildikten sonra nesnenin fotoraflama
- Otonom sürüş kontrol

🟧 **Beyza** — Araç Haritalandırma & Gazebo
-------------------------------------------
- Araç haritalandırması çözümleri
- Gazebo

🟨 **Mehmet** — Astrobiyoloji Görevi
-------------------------------------
- QR kod ile hedef tespiti
- Belirlenen QR kod bölgesine otonom olarak gitme
- PH ölçümü
- Otonom sürüş kontrolü

🟫 **Mehmet Emin** — Arayüz Geliştirme ve Manuel Kontrol
----------------------------------------------------------
- Yer istasyonu arayüzü için yazılan nodeların entegrasyonu
- Arayüz için eklenebilecek özelliklerinin araştırılması ve entegrasyonu
- Manuel sürüş kontrol
