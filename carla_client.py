"""
Purpose of carla_client.py:
Acts as a bridge between our 2D A* Path Planner and the 3D CARLA Simulator.

YENİ YAKLAŞIM:
  - Koniler yolun KENARINA dizilir (arabanın geçeceği koridor oluşur)
  - Araç yola (project_to_road) spawn edilir → roundabout'a düşmez
  - Rota CARLA yol waypoint'lerine göre takip edilir
"""
import carla
import math
import time


class CarlaBridge:
    def __init__(self, cell_size=2.0):
        self.cell_size = cell_size
        self.client   = None
        self.world    = None
        self.vehicle  = None
        self.spawned_obstacles = []

    # ------------------------------------------------------------------
    # BAĞLANTI
    # ------------------------------------------------------------------
    def connect(self):
        print("Connecting to CARLA Server...")
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world  = self.client.get_world()

            # Town05: En düz ve boş harita → ağaç/bina engeli olmaz
            self.world = self.client.load_world('Town05')

            print("Connected successfully!")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    # ------------------------------------------------------------------
    # YOL KORİDORU  —  koniler yolun iki yanına dizilir
    # ------------------------------------------------------------------
    def build_road_corridor(self, route_waypoints,
                            cone_spacing=3.5, cone_offset=2.5):
        """
        route_waypoints : carla.Waypoint listesi
        cone_spacing    : ardışık koniler arası min mesafe (m)
        cone_offset     : yol merkezinden yana koni uzaklığı (m)
                          → araç genişliği ~1.85 m, bu 2.5 m → ~0.65 m marjin her taraf
        """
        print("Yol koridoru inşa ediliyor (koniler kenara diziliyor)...")
        bp_lib  = self.world.get_blueprint_library()
        cone_bp = bp_lib.filter('static.prop.constructioncone')[0]

        last_loc = None

        for wp in route_waypoints:
            loc = wp.transform.location

            # Koniler arası minimum mesafe kontrolü (çok sık koyma)
            if last_loc:
                if math.hypot(loc.x - last_loc.x, loc.y - last_loc.y) < cone_spacing:
                    continue

            right = wp.transform.get_right_vector()

            for sign in [+1.0, -1.0]:   # sağ taraf ve sol taraf
                cx = loc.x + right.x * cone_offset * sign
                cy = loc.y + right.y * cone_offset * sign
                cz = loc.z + 0.1

                cone = self.world.try_spawn_actor(
                    cone_bp,
                    carla.Transform(carla.Location(x=cx, y=cy, z=cz))
                )
                if cone:
                    self.spawned_obstacles.append(cone)

            last_loc = loc

        print(f"Toplam {len(self.spawned_obstacles)} koni yerleştirildi!")

    # ------------------------------------------------------------------
    # ESKİ KODU KORUMAK İÇİN — grid tabanlı koni (artık kullanılmıyor)
    # ------------------------------------------------------------------
    def build_custom_maze(self, grid, offset_x=65.0, offset_y=65.0):
        """Grid tabanlı eski yöntem. Yeni projeler build_road_corridor kullanır."""
        print("(Eski yöntem) Grid tabanlı labirent inşa ediliyor...")
        bp_lib  = self.world.get_blueprint_library()
        cone_bp = bp_lib.filter('static.prop.constructioncone')[0]

        rows = len(grid)
        cols = len(grid[0])

        for x in range(rows):
            for y in range(cols):
                if grid[x][y] == 1:
                    wx = (x * self.cell_size) + offset_x
                    wy = (y * self.cell_size) + offset_y
                    sp = carla.Transform(carla.Location(x=wx, y=wy, z=0.2))
                    cone = self.world.try_spawn_actor(cone_bp, sp)
                    if cone:
                        self.spawned_obstacles.append(cone)

        print(f"Toplam {len(self.spawned_obstacles)} engel yerleştirildi!")

    # ------------------------------------------------------------------
    # YOL ROTA ÜRETICI  —  belirli mesafede waypoint dizisi oluşturur
    # ------------------------------------------------------------------
    def get_road_route(self, start_location, total_distance=200.0, step=2.0):
        """
        start_location : carla.Location — başlangıç noktasına yakın bir konum
        total_distance : rota uzunluğu (metre)
        step           : waypoint'ler arası mesafe (metre)
        Döndürür: carla.Waypoint listesi
        """
        carla_map = self.world.get_map()
        start_wp  = carla_map.get_waypoint(
            start_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if not start_wp:
            print("Başlangıç noktasına yakın yol bulunamadı!")
            return []

        waypoints   = [start_wp]
        current_wp  = start_wp
        accumulated = 0.0

        while accumulated < total_distance:
            next_wps = current_wp.next(step)
            if not next_wps:
                break
            current_wp   = next_wps[0]
            waypoints.append(current_wp)
            accumulated += step

        print(f"Rota üretildi: {len(waypoints)} waypoint, ~{accumulated:.0f} m")
        return waypoints

    # ------------------------------------------------------------------
    # ARAÇ SPAWN  —  yola yapıştırılır (roundabout'a düşmez)
    # ------------------------------------------------------------------
    def spawn_vehicle_on_road(self, start_location):
        """
        Aracı en yakın sürüş şeridine (project_to_road) spawn eder.
        Roundabout / fıskiye / platform gibi yapılara düşmeyi önler.
        """
        carla_map = self.world.get_map()
        spawn_wp  = carla_map.get_waypoint(
            start_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if spawn_wp:
            t = spawn_wp.transform
            t.location.z += 0.5          # yolun biraz üstüne
            print(f"Araç yola snap edildi: ({t.location.x:.1f}, {t.location.y:.1f})")
        else:
            print("Yakın yol bulunamadı, direkt koordinata spawn ediliyor.")
            t = carla.Transform(
                carla.Location(x=start_location.x, y=start_location.y, z=1.0)
            )

        bp_lib     = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, t)

        if self.vehicle:
            print("Araç spawn edildi. Fizik yerleşimi bekleniyor (2 sn)...")
            time.sleep(2.0)
        else:
            print("Araç spawn edilemedi! Koordinatları değiştirin.")

    # ESKİ SPAWN (grid tabanlı, geriye uyumluluk)
    def spawn_vehicle(self, start_grid_coord, offset_x=65.0, offset_y=65.0):
        target_x = (start_grid_coord[0] * self.cell_size) + offset_x
        target_y = (start_grid_coord[1] * self.cell_size) + offset_y

        spawn_transform = carla.Transform(
            carla.Location(x=target_x, y=target_y, z=2.0),
            carla.Rotation(pitch=0, yaw=90.0, roll=0)
        )
        bp_lib     = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)

        if self.vehicle:
            print(f"Araç grid konumuna spawn edildi: {spawn_transform.location}")
            print("Fizik yerleşimi bekleniyor (2 sn)...")
            time.sleep(2.0)
        else:
            print("Araç spawn edilemedi!")

    # ------------------------------------------------------------------
    # SÜRÜŞ  —  waypoint listesi boyunca ilerler
    # ------------------------------------------------------------------
    def drive_waypoints(self, route_waypoints, arrival_threshold=3.0):
        """
        Yol waypoint'leri boyunca aracı sürer.
        Takılma dedektörü (5 sn / 0.3 m) ve 10 sn konum logu dahil.
        """
        if not self.vehicle:
            print("Araç bulunamadı!")
            return

        print(f"Sürüş başlıyor — {len(route_waypoints)} hedef waypoint")
        last_log_time = time.time()

        for wp_idx, wp in enumerate(route_waypoints):
            target_x = wp.transform.location.x
            target_y = wp.transform.location.y

            stuck_timer    = time.time()
            stuck_last_loc = None

            while True:
                current_loc = self.vehicle.get_transform().location
                current_rot = self.vehicle.get_transform().rotation

                mesafe = math.hypot(target_x - current_loc.x,
                                    target_y - current_loc.y)

                # Hedefe yeterince yaklaştık → sonraki waypoint
                if mesafe < arrival_threshold:
                    break

                # ---- Takılma tespiti (5 sn / 0.3 m) ----
                if stuck_last_loc is None:
                    stuck_last_loc = (current_loc.x, current_loc.y)
                elif time.time() - stuck_timer >= 5.0:
                    moved = math.hypot(
                        current_loc.x - stuck_last_loc[0],
                        current_loc.y - stuck_last_loc[1]
                    )
                    if moved < 0.3:
                        print(f"[UYARI] WP {wp_idx} üzerinde takılı "
                              f"({current_loc.x:.1f}, {current_loc.y:.1f}) → atlanıyor")
                        break
                    stuck_last_loc = (current_loc.x, current_loc.y)
                    stuck_timer    = time.time()

                # ---- Direksiyon hesabı ----
                yaw_rad      = math.radians(current_rot.yaw)
                dx           = target_x - current_loc.x
                dy           = target_y - current_loc.y
                target_angle = math.atan2(dy, dx)

                angle_diff = target_angle - yaw_rad
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

                # [-π, +π] → [-1, +1] normalize
                steer    = max(-1.0, min(1.0, angle_diff / math.pi))
                throttle = max(0.20, 0.45 * (1.0 - 0.55 * abs(steer)))

                self.vehicle.apply_control(
                    carla.VehicleControl(throttle=throttle, steer=steer)
                )

                # ---- 10 saniyelik konum logu ----
                now = time.time()
                if now - last_log_time >= 10.0:
                    vel = self.vehicle.get_velocity()
                    spd = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6
                    print(
                        f"[LOG] WP {wp_idx}/{len(route_waypoints)} | "
                        f"Konum: ({current_loc.x:.1f}, {current_loc.y:.1f}) | "
                        f"Hedef: ({target_x:.1f}, {target_y:.1f}) | "
                        f"Mesafe: {mesafe:.1f} m | Hız: {spd:.1f} km/h | "
                        f"Steer: {steer:.3f} | Throttle: {throttle:.3f}"
                    )
                    last_log_time = now

                time.sleep(0.05)

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        print("Hedefe ulaşıldı! Araç durdu.")

    # Eski grid tabanlı sürüş (geriye uyumluluk)
    def drive_path(self, smooth_path, offset_x=65.0, offset_y=65.0):
        """Eski grid tabanlı sürüş. Yeni kod drive_waypoints kullanır."""
        if not self.vehicle:
            return

        print(f"(Eski yöntem) Grid path sürüşü — {len(smooth_path)} nokta")
        last_log_time = time.time()

        for wp_idx, point in enumerate(smooth_path):
            target_x = (point[0] * self.cell_size) + offset_x
            target_y = (point[1] * self.cell_size) + offset_y

            stuck_timer    = time.time()
            stuck_last_loc = None

            while True:
                current_loc = self.vehicle.get_transform().location
                current_rot = self.vehicle.get_transform().rotation
                mesafe      = math.hypot(target_x - current_loc.x,
                                         target_y - current_loc.y)

                if mesafe < 2.0:
                    break

                if stuck_last_loc is None:
                    stuck_last_loc = (current_loc.x, current_loc.y)
                elif time.time() - stuck_timer >= 5.0:
                    moved = math.hypot(
                        current_loc.x - stuck_last_loc[0],
                        current_loc.y - stuck_last_loc[1]
                    )
                    if moved < 0.3:
                        print(f"[UYARI] WP {wp_idx} takılı → atlanıyor")
                        break
                    stuck_last_loc = (current_loc.x, current_loc.y)
                    stuck_timer    = time.time()

                yaw_rad      = math.radians(current_rot.yaw)
                angle_diff   = math.atan2(target_y - current_loc.y,
                                          target_x - current_loc.x) - yaw_rad
                angle_diff   = (angle_diff + math.pi) % (2 * math.pi) - math.pi
                steer        = max(-1.0, min(1.0, angle_diff / math.pi))
                throttle     = max(0.20, 0.45 * (1.0 - 0.55 * abs(steer)))

                self.vehicle.apply_control(
                    carla.VehicleControl(throttle=throttle, steer=steer)
                )

                now = time.time()
                if now - last_log_time >= 10.0:
                    vel = self.vehicle.get_velocity()
                    spd = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) * 3.6
                    print(
                        f"[LOG] WP {wp_idx}/{len(smooth_path)} | "
                        f"Konum: ({current_loc.x:.1f}, {current_loc.y:.1f}) | "
                        f"Mesafe: {mesafe:.1f} m | Hız: {spd:.1f} km/h"
                    )
                    last_log_time = now

                time.sleep(0.05)

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        print("Hedefe ulaşıldı!")

    # ------------------------------------------------------------------
    # TEMİZLİK
    # ------------------------------------------------------------------
    def cleanup(self):
        if self.vehicle:
            self.vehicle.destroy()
        for obs in self.spawned_obstacles:
            obs.destroy()
        print("Simülasyon temizlendi.")