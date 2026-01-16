#!/usr/bin/env python3
"""
Drone Detection and Tracking System
مكتبات مطلوبة:
pip install numpy scipy matplotlib flask folium pyshark rtlsdr netifaces gpsd-py3
"""

import sys
import os
import time
import json
import threading
import queue
import numpy as np
from datetime import datetime
from collections import defaultdict

# ========== مكتبات الاختيارية ==========
try:
    from rtlsdr import RtlSdr
    SDR_AVAILABLE = True
except ImportError:
    SDR_AVAILABLE = False
    print("⚠️  مكتبة RTL-SDR غير مثبتة. سيتم تعطيل الكشف الراديوي")

try:
    import pyshark
    WIFI_AVAILABLE = True
except ImportError:
    WIFI_AVAILABLE = False
    print("⚠️  مكتبة PyShark غير مثبتة. سيتم تعطيل كشف Wi-Fi")

try:
    import folium
    from flask import Flask, render_template, jsonify
    WEB_AVAILABLE = True
except ImportError:
    WEB_AVAILABLE = False
    print("⚠️  مكتبات الويب غير مثبتة. سيتم تعطيل الواجهة الرسومية")

# ========== فئات النظام ==========

class DroneDetector:
    """فئة رئيسية لكشف الدرونز"""
    
    def __init__(self, config_file='config.json'):
        self.config = self.load_config(config_file)
        self.detected_drones = {}
        self.detection_history = []
        self.running = False
        
        # طابور للاتصال بين الخيوط
        self.detection_queue = queue.Queue()
        
        # إحصائيات
        self.stats = {
            'total_detections': 0,
            'unique_drones': set(),
            'last_update': datetime.now()
        }
        
        print("🚀 نظام كشف الدرونز جاهز للبدء")
    
    def load_config(self, config_file):
        """تحميل إعدادات النظام"""
        default_config = {
            'sdr': {
                'enabled': True,
                'frequency': 2.4e9,  # 2.4 GHz
                'sample_rate': 2.4e6,
                'gain': 'auto',
                'ppm': 0
            },
            'wifi': {
                'enabled': True,
                'interface': 'wlan1',
                'monitor_mode': True,
                'channel': 6
            },
            'detection': {
                'signal_threshold': -60,  # dBm
                'min_duration': 2,  # ثواني
                'update_interval': 1  # ثانية
            },
            'map': {
                'default_location': [24.7136, 46.6753],  # الرياض
                'default_zoom': 12,
                'update_interval': 2  # ثواني
            },
            'web': {
                'host': '0.0.0.0',
                'port': 8080,
                'debug': False
            }
        }
        
        if os.path.exists(config_file):
            with open(config_file, 'r') as f:
                user_config = json.load(f)
                # دمج الإعدادات
                for key in user_config:
                    if key in default_config:
                        default_config[key].update(user_config[key])
        
        return default_config
    
    def start_sdr_detection(self):
        """بدء الكشف باستخدام RTL-SDR"""
        if not SDR_AVAILABLE:
            print("❌ RTL-SDR غير متاح")
            return
        
        print("📡 بدء الكشف الراديوي...")
        
        try:
            sdr = RtlSdr()
            sdr.sample_rate = self.config['sdr']['sample_rate']
            sdr.center_freq = self.config['sdr']['frequency']
            sdr.gain = self.config['sdr']['gain']
            sdr.ppm_error = self.config['sdr']['ppm']
            
            def sdr_callback(samples, context):
                """معالجة العينات من SDR"""
                power = 10 * np.log10(np.mean(np.abs(samples)**2))
                
                if power > self.config['detection']['signal_threshold']:
                    freq = sdr.center_freq / 1e6  # تحويل إلى MHz
                    drone_id = f"RF_{int(freq)}_{int(time.time())}"
                    
                    detection = {
                        'id': drone_id,
                        'type': 'RF_SIGNAL',
                        'frequency': freq,
                        'power': power,
                        'timestamp': datetime.now().isoformat(),
                        'source': 'SDR',
                        'location': self.estimate_location(freq, power)
                    }
                    
                    self.detection_queue.put(detection)
                    print(f"📡 إشارة راديوية قوية: {power:.1f} dBm @ {freq:.1f} MHz")
            
            # بدء الاستقبال
            sdr.read_samples_async(sdr_callback, 256*1024)
            
            while self.running:
                time.sleep(0.1)
            
            sdr.cancel_read_async()
            sdr.close()
            
        except Exception as e:
            print(f"❌ خطأ في SDR: {e}")
    
    def start_wifi_detection(self):
        """بدء الكشف عبر Wi-Fi"""
        if not WIFI_AVAILABLE:
            print("❌ PyShark غير متاح")
            return
        
        print("📶 بدء كشف شبكات Wi-Fi...")
        
        try:
            interface = self.config['wifi']['interface']
            
            # تصفية حزم Wi-Fi المشبوهة
            display_filter = 'wlan.fc.type_subtype == 0x08 || wlan.fc.type_subtype == 0x05'
            
            capture = pyshark.LiveCapture(
                interface=interface,
                display_filter=display_filter,
                use_json=True
            )
            
            known_drone_ouis = {
                '90:3a:e6': 'DJI',
                '60:60:1f': 'DJI',
                'a0:14:3d': 'Parrot',
                '90:03:b7': 'Parrot',
                '00:12:1c': 'Yuneec'
            }
            
            for packet in capture.sniff_continuously():
                if not self.running:
                    break
                
                try:
                    if hasattr(packet, 'wlan'):
                        bssid = packet.wlan.bssid.replace(':', '').lower()
                        
                        # التحقق من OUI المصنع
                        oui = bssid[:6]
                        if oui in known_drone_ouis:
                            drone_type = known_drone_ouis[oui]
                            ssid = packet.wlan.ssid if hasattr(packet.wlan, 'ssid') else 'Unknown'
                            
                            signal_strength = int(packet.wlan_radio.signal_dbm)
                            
                            if signal_strength > self.config['detection']['signal_threshold']:
                                drone_id = f"WIFI_{bssid[-6:]}"
                                
                                detection = {
                                    'id': drone_id,
                                    'type': drone_type,
                                    'ssid': ssid,
                                    'bssid': bssid,
                                    'power': signal_strength,
                                    'timestamp': datetime.now().isoformat(),
                                    'source': 'Wi-Fi',
                                    'channel': int(packet.wlan_radio.channel),
                                    'location': self.triangulate_wifi_position(bssid, signal_strength)
                                }
                                
                                self.detection_queue.put(detection)
                                print(f"📶 درون {drone_type} مكتشف: {ssid} ({signal_strength} dBm)")
                
                except AttributeError:
                    continue
        
        except Exception as e:
            print(f"❌ خطأ في كشف Wi-Fi: {e}")
    
    def estimate_location(self, frequency, power):
        """تقدير الموقع بناء على التردد والقوة (محاكاة)"""
        # في الواقع، هذا يتطلب مصفوفة هوائيات أو تثليث
        # هذه محاكاة لأغراض العرض فقط
        
        base_lat, base_lon = self.config['map']['default_location']
        
        # محاكاة موقع عشوائي حول المركز
        import random
        lat_offset = random.uniform(-0.01, 0.01)
        lon_offset = random.uniform(-0.01, 0.01)
        
        return {
            'latitude': base_lat + lat_offset,
            'longitude': base_lon + lon_offset,
            'accuracy': random.randint(10, 100)  # دقة بالأمتار
        }
    
    def triangulate_wifi_position(self, bssid, signal_strength):
        """تثليث موقع الجهاز Wi-Fi (محاكاة)"""
        # نظام حقيقي يتطلب نقاط وصول متعددة
        return self.estimate_location(2400, signal_strength)
    
    def process_detections(self):
        """معالجة الاكتشافات من الطابور"""
        while self.running:
            try:
                detection = self.detection_queue.get(timeout=1)
                
                drone_id = detection['id']
                
                # تحديث أو إضافة الدرون
                if drone_id in self.detected_drones:
                    # تحديث المدة والتاريخ
                    self.detected_drones[drone_id]['last_seen'] = detection['timestamp']
                    self.detected_drones[drone_id]['duration'] += 1
                    self.detected_drones[drone_id]['detection_count'] += 1
                else:
                    # إضافة درون جديد
                    detection['first_seen'] = detection['timestamp']
                    detection['duration'] = 1
                    detection['detection_count'] = 1
                    self.detected_drones[drone_id] = detection
                    self.stats['unique_drones'].add(drone_id)
                
                # إضافة للسجل
                self.detection_history.append(detection)
                self.stats['total_detections'] += 1
                
                # الاحتفاظ بأحدث 1000 اكتشاف فقط
                if len(self.detection_history) > 1000:
                    self.detection_history = self.detection_history[-1000:]
                
                print(f"✅ درون مكتشف: {detection.get('type', 'Unknown')} - {drone_id}")
            
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ خطأ في معالجة الاكتشاف: {e}")
    
    def generate_map(self, filename='drone_map.html'):
        """إنشاء خريطة HTML تفاعلية"""
        if not WEB_AVAILABLE:
            print("❌ مكتبات الخرائط غير متوفرة")
            return
        
        print("🗺️  إنشاء الخريطة...")
        
        # مركز الخريطة
        center_lat, center_lon = self.config['map']['default_location']
        
        # إنشاء الخريطة
        m = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=self.config['map']['default_zoom'],
            tiles='OpenStreetMap'
        )
        
        # إضافة علامات للدرونز
        for drone_id, drone in self.detected_drones.items():
            if 'location' in drone:
                lat = drone['location']['latitude']
                lon = drone['location']['longitude']
                
                # تحديد لون العلامة حسب نوع الدرون
                color = 'red'
                if 'DJI' in str(drone.get('type', '')):
                    color = 'blue'
                elif 'Parrot' in str(drone.get('type', '')):
                    color = 'green'
                
                # نص المنبثقة
                popup_text = f"""
                <b>🛸 درون #{drone_id}</b><br>
                النوع: {drone.get('type', 'غير معروف')}<br>
                القوة: {drone.get('power', 'N/A')} dBm<br>
                المصدر: {drone.get('source', 'N/A')}<br>
                أول اكتشاف: {drone.get('first_seen', 'N/A')}<br>
                آخر ظهور: {drone.get('last_seen', 'N/A')}<br>
                المدة: {drone.get('duration', 0)} ثانية
                """
                
                # إضافة العلامة
                folium.Marker(
                    [lat, lon],
                    popup=popup_text,
                    tooltip=f"درون {drone.get('type', '')}",
                    icon=folium.Icon(color=color, icon='drone', prefix='fa')
                ).add_to(m)
        
        # إضافة دائرة للدقة
        for drone_id, drone in self.detected_drones.items():
            if 'location' in drone:
                lat = drone['location']['latitude']
                lon = drone['location']['longitude']
                accuracy = drone['location'].get('accuracy', 50)
                
                folium.Circle(
                    location=[lat, lon],
                    radius=accuracy,
                    color='crimson',
                    fill=True,
                    fill_color='crimson',
                    fill_opacity=0.2,
                    popup=f"دقة: ±{accuracy} متر"
                ).add_to(m)
        
        # حفظ الخريطة
        m.save(filename)
        print(f"✅ تم حفظ الخريطة في: {filename}")
        
        return filename
    
    def start_web_server(self):
        """تشغيل خادم ويب للواجهة الرسومية"""
        if not WEB_AVAILABLE:
            print("❌ Flask غير متاح")
            return
        
        app = Flask(__name__)
        
        @app.route('/')
        def index():
            """الصفحة الرئيسية"""
            return render_template('''
            <!DOCTYPE html>
            <html>
            <head>
                <title>🚁 نظام مراقبة الدرونز</title>
                <meta charset="utf-8">
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/leaflet.css" />
                <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css" />
                <style>
                    body { margin: 0; padding: 0; font-family: Arial, sans-serif; }
                    #map { height: 70vh; width: 100%; }
                    #dashboard { padding: 20px; background: #f5f5f5; }
                    .stats { display: flex; gap: 20px; flex-wrap: wrap; }
                    .stat-box { 
                        background: white; 
                        padding: 15px; 
                        border-radius: 8px; 
                        box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                        min-width: 200px;
                    }
                    .drone-list { margin-top: 20px; }
                    .drone-item {
                        background: white;
                        padding: 10px;
                        margin: 5px 0;
                        border-left: 4px solid #007bff;
                        border-radius: 4px;
                    }
                </style>
            </head>
            <body>
                <div id="dashboard">
                    <h1><i class="fas fa-drone"></i> نظام مراقبة الدرونز</h1>
                    <div class="stats">
                        <div class="stat-box">
                            <h3><i class="fas fa-broadcast-tower"></i> الإحصائيات</h3>
                            <p>الدرونز النشطة: <span id="active-drones">0</span></p>
                            <p>إجمالي الاكتشافات: <span id="total-detections">0</span></p>
                            <p>آخر تحديث: <span id="last-update">--</span></p>
                        </div>
                        <div class="stat-box">
                            <h3><i class="fas fa-cogs"></i> التحكم</h3>
                            <button onclick="refreshMap()"><i class="fas fa-sync-alt"></i> تحديث الخريطة</button>
                            <button onclick="exportData()"><i class="fas fa-download"></i> تصدير البيانات</button>
                        </div>
                    </div>
                    <div class="drone-list">
                        <h3><i class="fas fa-list"></i> الدرونز المكتشفة</h3>
                        <div id="drone-list-container"></div>
                    </div>
                </div>
                <div id="map"></div>
                
                <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/leaflet.js"></script>
                <script>
                    var map = L.map('map').setView([24.7136, 46.6753], 12);
                    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                        attribution: '© OpenStreetMap contributors'
                    }).addTo(map);
                    
                    var droneMarkers = {};
                    
                    function updateMap(drones) {
                        // إزالة العلامات القديمة
                        for (var id in droneMarkers) {
                            map.removeLayer(droneMarkers[id]);
                        }
                        droneMarkers = {};
                        
                        // إضافة علامات جديدة
                        drones.forEach(function(drone) {
                            if (drone.location) {
                                var marker = L.marker([drone.location.latitude, drone.location.longitude])
                                    .bindPopup(`<b>🛸 ${drone.type || 'درون'}</b><br>
                                               القوة: ${drone.power} dBm<br>
                                               المصدر: ${drone.source}<br>
                                               المدة: ${drone.duration}s`);
                                
                                droneMarkers[drone.id] = marker;
                                marker.addTo(map);
                            }
                        });
                    }
                    
                    function updateDashboard(stats) {
                        document.getElementById('active-drones').textContent = stats.active;
                        document.getElementById('total-detections').textContent = stats.total;
                        document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
                        
                        // تحديث قائمة الدرونز
                        var container = document.getElementById('drone-list-container');
                        container.innerHTML = '';
                        
                        stats.drones.forEach(function(drone) {
                            var div = document.createElement('div');
                            div.className = 'drone-item';
                            div.innerHTML = `
                                <strong>${drone.type || 'درون'}</strong><br>
                                ID: ${drone.id}<br>
                                القوة: ${drone.power} dBm | المدة: ${drone.duration}s
                            `;
                            container.appendChild(div);
                        });
                    }
                    
                    function refreshData() {
                        fetch('/api/drones')
                            .then(response => response.json())
                            .then(data => {
                                updateMap(data.drones);
                                updateDashboard({
                                    active: data.drones.length,
                                    total: data.total_detections,
                                    drones: data.drones
                                });
                            });
                    }
                    
                    function refreshMap() {
                        fetch('/api/update_map')
                            .then(response => response.json())
                            .then(data => {
                                if (data.map_url) {
                                    window.open(data.map_url, '_blank');
                                }
                            });
                    }
                    
                    function exportData() {
                        fetch('/api/export')
                            .then(response => response.blob())
                            .then(blob => {
                                var url = window.URL.createObjectURL(blob);
                                var a = document.createElement('a');
                                a.href = url;
                                a.download = 'drones_export.json';
                                a.click();
                            });
                    }
                    
                    // تحديث تلقائي كل 3 ثواني
                    setInterval(refreshData, 3000);
                    refreshData(); // التشغيل الأولي
                </script>
            </body>
            </html>
            ''')
        
        @app.route('/api/drones')
        def api_drones():
            """واجهة برمجية للدرونز"""
            drones_list = list(self.detected_drones.values())
            return jsonify({
                'drones': drones_list,
                'total_drones': len(drones_list),
                'total_detections': self.stats['total_detections'],
                'timestamp': datetime.now().isoformat()
            })
        
        @app.route('/api/update_map')
        def api_update_map():
            """تحديث الخريطة"""
            filename = self.generate_map()
            return jsonify({
                'status': 'success',
                'map_url': f'/{filename}',
                'timestamp': datetime.now().isoformat()
            })
        
        @app.route('/api/export')
        def api_export():
            """تصدير البيانات"""
            export_data = {
                'detected_drones': self.detected_drones,
                'detection_history': self.detection_history[-100:],  # آخر 100 اكتشاف
                'stats': self.stats,
                'export_time': datetime.now().isoformat()
            }
            
            return jsonify(export_data)
        
        @app.route('/drone_map.html')
        def serve_map():
            """خدمة ملف الخريطة"""
            self.generate_map()
            with open('drone_map.html', 'r') as f:
                return f.read()
        
        print(f"🌐 خادم الويب يعمل على: http://{self.config['web']['host']}:{self.config['web']['port']}")
        app.run(
            host=self.config['web']['host'],
            port=self.config['web']['port'],
            debug=self.config['web']['debug'],
            use_reloader=False
        )
    
    def start(self):
        """بدء النظام كاملاً"""
        print("🚀 بدء نظام كشف وتتبع الدرونز...")
        self.running = True
        
        # خيوط المعالجة
        threads = []
        
        # خيط لمعالجة الاكتشافات
        process_thread = threading.Thread(target=self.process_detections)
        process_thread.daemon = True
        threads.append(process_thread)
        process_thread.start()
        
        # خيط لـ SDR
        if self.config['sdr']['enabled'] and SDR_AVAILABLE:
            sdr_thread = threading.Thread(target=self.start_sdr_detection)
            sdr_thread.daemon = True
            threads.append(sdr_thread)
            sdr_thread.start()
        
        # خيط لـ Wi-Fi
        if self.config['wifi']['enabled'] and WIFI_AVAILABLE:
            wifi_thread = threading.Thread(target=self.start_wifi_detection)
            wifi_thread.daemon = True
            threads.append(wifi_thread)
            wifi_thread.start()
        
        # خيط لخادم الويب
        if WEB_AVAILABLE:
            web_thread = threading.Thread(target=self.start_web_server)
            web_thread.daemon = True
            threads.append(web_thread)
            web_thread.start()
        
        print("✅ جميع الأنظمة تعمل. اضغط Ctrl+C لإيقاف النظام.")
        
        try:
            # الحلقة الرئيسية
            while self.running:
                # تحديث الإحصائيات كل 10 ثواني
                time.sleep(10)
                self.stats['last_update'] = datetime.now()
                
                # إنشاء خريطة تلقائية كل دقيقة
                if int(time.time()) % 60 == 0:
                    self.generate_map()
                
                # عرض الإحصائيات
                print(f"\n📊 الإحصائيات:")
                print(f"   الدرونز النشطة: {len(self.detected_drones)}")
                print(f"   إجمالي الاكتشافات: {self.stats['total_detections']}")
                print(f"   الدرونز الفريدة: {len(self.stats['unique_drones'])}")
                print(f"   آخر تحديث: {self.stats['last_update'].strftime('%H:%M:%S')}")
        
        except KeyboardInterrupt:
            print("\n🛑 إيقاف النظام...")
            self.stop()
    
    def stop(self):
        """إيقاف النظام"""
        self.running = False
        print("✅ النظام متوقف.")
        
        # حفظ البيانات
        self.save_data()
        
        # إنشاء خريطة نهائية
        if WEB_AVAILABLE:
            self.generate_map('final_drone_map.html')
    
    def save_data(self):
        """حفظ البيانات في ملف"""
        data = {
            'detected_drones': self.detected_drones,
            'detection_history': self.detection_history,
            'stats': {
                'total_detections': self.stats['total_detections'],
                'unique_drones': list(self.stats['unique_drones']),
                'last_update': self.stats['last_update'].isoformat()
            },
            'system_info': {
                'stop_time': datetime.now().isoformat(),
                'runtime': time.time() - getattr(self, 'start_time', time.time())
            }
        }
        
        filename = f'drone_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2, default=str)
        
        print(f"💾 تم حفظ البيانات في: {filename}")

# ========== البرنامج الرئيسي ==========

def main():
    """الدالة الرئيسية"""
    print("""
    ██████╗ ██████╗  ██████╗ ███╗   ██╗███████╗
    ██╔══██╗██╔══██╗██╔═══██╗████╗  ██║██╔════╝
    ██║  ██║██████╔╝██║   ██║██╔██╗ ██║█████╗  
    ██║  ██║██╔══██╗██║   ██║██║╚██╗██║██╔══╝  
    ██████╔╝██║  ██║╚██████╔╝██║ ╚████║███████╗
    ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝
    
    نظام كشف وتتبع الطائرات بدون طيار
    الإصدار: 2.0 | المطور: مهندس أنظمة متكاملة
    """)
    
    # التحقق من الصلاحيات
    if os.geteuid() != 0:
        print("⚠️  تحذير: يفضل تشغيل البرنامج بصلاحيات root")
        print("   sudo python3 drone_detector.py")
    
    # إنشاء النظام
    detector = DroneDetector()
    
    # بدء التشغيل
    try:
        detector.start()
    except Exception as e:
        print(f"❌ خطأ غير متوقع: {e}")
        detector.stop()

if __name__ == "__main__":
    main()
