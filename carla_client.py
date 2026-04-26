"""
Purpose of carla_client.py:
Acts as a bridge between our 2D A* Path Planner and the 3D CARLA Simulator.
Converts 2D grid coordinates to 3D world coordinates and drives the vehicle.
"""
import carla
import math
import time


class CarlaBridge:
    def __init__(self, cell_size=2.0):
        self.cell_size = cell_size
        self.client = None
        self.world = None
        self.vehicle = None
        self.spawned_obstacles = []  # Çıkarken konileri temizlemek için

    def connect(self):
        print("Connecting to CARLA Server...")
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()

            # Daha boş ve düz bir harita olan Town05'i yükleyelim (Opsiyonel)
            # self.world = self.client.load_world('Town05')

            print("Connected successfully!")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def build_custom_maze(self, grid, offset_x=100.0, offset_y=150.0):
        """Senin A* grid'indeki engelleri (1'leri) 3D konilere dönüştürür"""
        print("3D Labirent inşa ediliyor...")
        blueprint_library = self.world.get_blueprint_library()

        # CARLA'nın içindeki standart trafik konisi objesi
        cone_bp = blueprint_library.filter('static.prop.constructioncone')[0]

        # Grid matrisini tara
        rows = len(grid)
        cols = len(grid[0])

        for x in range(rows):
            for y in range(cols):
                if grid[x][y] == 1:  # Eğer bu hücre engel ise
                    # 3D koordinata çevir
                    world_x = (x * self.cell_size) + offset_x
                    world_y = (y * self.cell_size) + offset_y

                    # Koniyi yere (z=0.5) yerleştir
                    spawn_point = carla.Transform(carla.Location(x=world_x, y=world_y, z=0.2))
                    cone = self.world.try_spawn_actor(cone_bp, spawn_point)

                    if cone:
                        self.spawned_obstacles.append(cone)
        print(f"Toplam {len(self.spawned_obstacles)} adet engel yerleştirildi!")

    def spawn_vehicle(self, start_grid_coord, offset_x=100.0, offset_y=150.0):
        # Mıknatısı (project_to_road) kapattık! Direkt kendi labirentimize ışınlanıyoruz.
        target_x = (start_grid_coord[0] * self.cell_size) + offset_x
        target_y = (start_grid_coord[1] * self.cell_size) + offset_y

        # Arabayı konilerin arasına, havadan (z=2.0) yavaşça bırakıyoruz
        spawn_transform = carla.Transform(
            carla.Location(x=target_x, y=target_y, z=2.0),
            carla.Rotation(pitch=0, yaw=90.0, roll=0)
        )

        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)

        if self.vehicle:
            print(f"Araç labirentin başlangıç noktasına başarıyla indirildi: {spawn_transform.location}")
        else:
            print("Araç indirilemedi! O koordinatta devasa bir bina veya obje olabilir. Offset'leri değiştirin.")

    def drive_path(self, smooth_path, offset_x=100.0, offset_y=150.0):
        if not self.vehicle:
            return

        print("A* Algoritması devrede, Kör Sürüş başlıyor...")

        for point in smooth_path:
            target_x = (point[0] * self.cell_size) + offset_x
            target_y = (point[1] * self.cell_size) + offset_y

            # YENİ EKLENEN KISIM: Hedefe varana kadar bu döngüde kal
            while True:
                current_loc = self.vehicle.get_transform().location
                current_rot = self.vehicle.get_transform().rotation

                # Arabanın hedefe olan kuş uçuşu mesafesini hesapla
                mesafe = math.hypot(target_x - current_loc.x, target_y - current_loc.y)

                # EĞER HEDEFE 1.5 METREDEN FAZLA YAKLAŞTIYSAK, SIRADAKİ NOKTAYA GEÇ!
                # (Eğer virajı hala erken dönüyorsa bu sayıyı 1.0 veya 0.5 yapabilirsin)
                if mesafe < 1.5:
                    break

                # Yönlendirme (Direksiyon) hesaplamaları
                yaw_rad = math.radians(current_rot.yaw)
                dx = target_x - current_loc.x
                dy = target_y - current_loc.y
                target_angle = math.atan2(dy, dx)

                angle_diff = target_angle - yaw_rad
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

                steer = max(-1.0, min(1.0, angle_diff))

                # Yavaş ve kontrollü bir sürüş (%35 gaz)
                control = carla.VehicleControl(throttle=0.35, steer=steer)
                self.vehicle.apply_control(control)

                # Bekleme süresini kısalttık ki araba daha sık tepki versin
                time.sleep(0.05)

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        print("Hedefe başarıyla ulaşıldı!")

    def cleanup(self):
        if self.vehicle:
            self.vehicle.destroy()
        for obstacle in self.spawned_obstacles:
            obstacle.destroy()
        print("Simülasyon temizlendi.")