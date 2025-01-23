# 1. Genel Paketleme Özeti

## 1.1 `moria_bringup`
Bu paket robotun direkt rviz ortamında açılmasını sağlamak ve diğer kullanıcı paketlere koordinat transformlarını iletme amacı için yapılmıştır.

## 1.2 `moria_cartographer`
Bu paket 2B ve 3B haritalama algoritmalarını barındırır. Paketin kullanımı aşağıda detaylı belirtilmiştir.

## 1.3 `moria_description`
Bu paket robotun URDF tanımlamalarını içerir. Ana dosyalar `moria.urdf.xacro` ve `moria_fixed.urdf.xacro`. 

- `moria.urdf.xacro`: Robotun kollarının hareketli tanımladığı dosyadır. Rviz için bu dosya kullanılmaktadır. 
- `moria_fixed.urdf.xacro`: Robotun kollarının sabit tanımlandığı dosyadır. Gazebo simülasyon ortamı için hazırlanmıştır.

## 1.4 `moria_gazebo`
Bu paket tanımlanan robotun Gazebo ortamında kullanılan sürüş kontrolleri, sensör tanımlamaları ve pluginlerinin tanımlandığı dosyaları, oluşturulan dünyaları ve gerekli parametre ve python dosyalarını içermektedir.



# 2. Kurulum
1. ROS2 Humle kurulacak
2. Aşağıda belirtilen kütüphaneler kurulacak
3. `workspace` klasörü ve içerisinde `src` klasörü oluşturulacak 
4. `moria_bringup`, `moria_cartographer`, `moria_description` ve `moria_gazebo` klasörleri `workspace/src/` içine atılacak
5. workspace klasörü içerisindeyken paketler kurulacak
6. source işlemi yapılacak

## 2.1 Rviz Açmak için
`ros2 launch moria_bringup rviz2.launch.py`

## 2.2 Modeli Gazeboda Açmak İçin
`ros2 launch moria_gazebo <script.py>

**Kullanılabilecek Scriptler**
1. `btu_big.launch.py`: G Blok Kat 3 için yapılmış 1:1 ölçekli haritada robotu açar.
2. `btu.launch.py`: G Blok Kat 3 için yapılmış 1:10 ölçekli haritada robotu açar.
3. `empty_world.launch.py`: Boş bir dünyada robotu açar.

## 2.3 Haritalama İçin
1. Robot dünyada açılacak
2. Haritalama algoritması çalıştırılacak
   1. slam_toolbox asynchron
   2. vslam
3. Duvar takibi algoritması başlatılacak
4. Duvar takibi bittikten sonra keşif algoritması başlatılacak
5. Harita Nav2 kütüphanesi kullanılarak kaydedilecek.



# 3. Paket Açıklamaları
## 3.1 `moria_bringup`
`robot_state_publisher` kütüphanesi kullanılarak robotun koordinat transformlarını yayınlar.

## 3.2 `moria_cartographer`
2B ve 3B haritalama yapmak için hazırlanmıştır.

Gerekli ROS Kütüphaneleri (Versiyonlar "dependencies" dosyasının içerisinde bulunmaktadır.):

   - Twist Mux
   - Slam_toolbox
   - Navigation2
   - RTabMap
   - Numpy
   - Scikit-learn
   - Open3D
   - PIL
   - yaml
   - gazebo-ros
   - tf2_ros

**SLAM işlemi için yapılması gerekenler:**

   - Moria gazebo ortamına spawnlanmalıdır. `ros2 launch moria_gazebo btu.launch.py`
   - Slam_toolbox çalıştırılmalıdır. `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True`
   - Duvar takibi algoritması çalıştırılmalıdır. `ros2 run moria_cartographer wall_track.py`
   - Duvar takibi tamamlandıktan sonra eksik kalan yerler frontier exploration algoritması ile tamamlanmalıdır. `ros2 run moria_cartographer frontier_exploration.py`
   - 2D Harita nav2 kullanılarak kaydedilmelidir. `ros2 run nav2_map_server map_saver_cli -f my_map`

**VSLAM işlemi için yapılması gerekenler:**

   - Moria gazebo ortamına spawnlanmalıdır.
   - Moria VSLAM çalıştırılmalıdır `ros2 launch moria_cartographer moria_vslam.launch.py`
   - Duvar takibi algoritması çalıştırılmalıdır.
   - Frontier exploration algoritması çalıştırılmalıdır.
   - Haritalama işlemi tamamlandığında process terminalden kapatılarak 3D harita kaydedilmelidir <CTRL + C>

**VSLAM - SLAM işlemi için yapılması gerekenler:**

   - Moria gazebo ortamına spawnlanmalıdır.
   - Slam_toolbox çalıştırılmalıdır.
   - Moria VSLAM çalıştırılmalıdır.
   - Duvar takibi algoritması çalıştırılmalıdır.
   - Frontier exploration algoritması çalıştırılmalıdır.
   - 2D Harita nav2 kullanılarak kaydedilmelidir.
   - Haritalama işlemi tamamlandığında process terminalden kapatılarak 3D harita kaydedilmelidir. 

Yapılan bu haritalama işlemleri sırasında sürecin takibi RViz üzerinden yapılabilmektedir.

Slam_toolbox kütüphanesi "/map" konusundan harita yayınlamaktadır.
RTabMap kütüphanesine ait 3D map "/map_graph" konusundan yayınlanmaktadır, 2D harita ise "/map2" konusundan yayınlanmaktadır.

### 3.2.1 Frontier Exploration
Frontier_Exploration algoritması bir çeşit haritalama stratejisidir ve Moria'nın daha etkili haritalama yapması için kullanılmaktadır.


**Çalışma Prensibi:**

   - Haritalama işlemi başladığı anda oluşan ilk harita "kanıt haritası" olarak alınmaktadır.
   - Kanıt haritası üzerinde keşfedilmemiş pikseller komşuluk ilişkilerine bakılarak bir ağırlıklı toplam ile puanlanır.
   - Keşfedilmeye en uygun piksel "best_frontier" olarak işaretlenir.
   - İşaretlenen piksele engellerden kaçınarak ilerleyebilmek için NAV2 kütüphanesi kullanılarak rota planlama işlemi yapılır.
   - Moria istenilen piksele ulaştığında bu işlem tekrarlanır ve bu sayede haritalama yapılır.

NAV2 Kütüphanesi vanilla olarak "Djikstra" rota planlama algoritmasını kullanmaktadır. Bu algoritma kesinlikle en kısa yolu vermektedir ancak çok yavaş bir algoritmadır.
Bu sebeple "Moria" yine NAV2 Kütüphanesinin içerisinde bulunan "Hybrid A*" rota planlama algoritmasını kullanmaktadır.

### 3.2.2 Trim 
Trim algoritması lidar tarafından algılanması mümkün olmayan ve hayali duvar oluşumu gibi bir çok hataya sebep olabilecek cam, ayna vb. yüzeylerin kamera ile tespit edilmesini sağlamaktadır.

**Gerekli Kütüphaneler:**

   - CV2
   - Numpy
   - CV_Bridge

**Çalışma Prensibi:**
	
   - *Camera_callback* fonksiyonu ile kameradan (/camera_rgb/image_raw) RGB görüntü alınmaktadır.
   - *Detect_glass_regions* fonksiyonu ile görüntü grayscale formata çevirilmekte ve Canny algoritması kullanılarak yüksek kontrastlı kenar tespiti yapılmaktadır. Düşük kontrastlı bölgeler Laplace dönüşümü ile analiz edilir ve filtrelenir. Son 	  olarak tespit edilen cam (reflector) yüzeylere ait x,y,w,h bilgileri döndürülür.
   - *map_lidar_to_camera* fonksiyonu ile lidar noktalarının kamera görüntüsündeki konumu hesaplanır.
   - Son olarak ise lidar verileri güncellenerek /filtered_scan ROS Topic'i altında yayınlanır.

Haritalanması gereken bölge cam, ayna vb. objeler bulundurmuyorsa veya LİDAR sensörünün ulaşamayacağı yükseklikte ise bu scripti kullanmaya gerek yoktur. Cam, ayna vb. objeler bulunmayan bir ortamda bu scripti kullanmak haritalama performansını kötü etkileyebilir.

### 3.2.3 Wall Tracking
Duvar takibi algoritması sadece lidar sensör verileri kullanılarak gerçek zamanlı bir control algoritmasıdır. Olabilecek en basit şekilde kullanılan PD kontrol ile lidardan her veri için hesaplamalar yapıp robotu duvara 1.41m uzaklıkta tutmayı sağlamaktadır. 

**Gerekli Kütüphaneler**
   - rclpy
   - math
   - numpy
   - sensor_msgs
   - nav_msgs
   - tf2_ros
   - geometry_msgs
   - /map topic

**Çalışma Prensibi**
    - Yayınlanan ros topic arasında "/map" yayınlanmasını bekler.
    - Robotun konumu ve yönü hesaplanır. Bu işlem sadece kullanıcıyı bildirmek amacıyla yapılır. Hareket algoritmasında yeri yoktur.
    - Yakında duvar olup olmadığı kontrol edilir.
      - Eğer duvar varsa duvarı sağına alır.
      - Eğer duvar yoksa 0.7m mesafede duvar görene kadar ileri hareket gerçekleştirilir.
    - Duvar bulunduktan sonra 180 derece tarama yapan lidarın 45. indeksindeki veriye göre hata PD kontrolcüsüne gönderilip robotun açısal hızı belirlenir.
    - Sağa ve sola dönüşleri sadece bu işlemleri kullanarak robot duvarları takip eder.


## `moria_description`
Robotun tasarlanmış parçalarını ve tanımlamalarını içerir. Tanımlamalar iki farklı durum için iki farklı ana dosya üzerinden yapılmıştır. Xacro formatı kullanılmıştır. 

## `moria_gazebo`
Robotun gazebo ortamında oluşturulması sağlaması için yapılmıştır. 
- `config/`: gerekli yaml parametre dosyalarını barındırır.
- `launch/`: Robotu istenilen simülasyon ortamında açmak için kullanılır. Bu haritalama paketi için gereklidir.
  - `btu_big.launch.py`: Hazırlanmış 1:1 ölçekli haritada robotu açar.
  - `btu.launch.py`: Hazırlanmış 1:10 ölçekli haritada robotu açar.
  - `empty_world.launch.py`: Boş bir dünyada robotu açmak için kullanılır.
  - `spawn_moria.launch.py`: Moria tasarım dosyalarını belirterek gazebo ortamına getirilmesini sağlar.
  - `state_publisher.launch.py`: `models/moria/model.urdf` dosyasını okuyup gerekli pluginleri ve sensörlerin çalışma prensibini gazeboya iletmek için kullanılır.
- `models/`: 1:1 ölçekli harita ve moria modellerini tanımlamalarıyla birlikte barındırır.
- `rviz/`: Simülasyonun yanında rviz uygulaması için parametreleri barındırır.
- `worlds/`:  Kullanılacak haritaların ana tanımlamalarını içerir. İçerisindeki dosyalar gerekli dosyaları bulup çağırmaktadır.


Eksikler:

	- 3D Haritalama sırasında kameranın görüş açısı ve konumu sebebiyle 2 adet kör nokta bulunmaktadır. Bu kör noktalar 3D harita 	  üzerinde boş kalmış duvar parçalarıdır.

Çözüm Önerisi:

	- Kör noktada bulunan duvarların haritalanması işlemi için, duvar takibi algoritmasını bir defada sol tarafı takip edecek 	  şekilde çalıştırmak yeterli olacaktır.


Gelecekte Yapılması İstenilen/Planlanan İşlemler:

	- Haritası çıkartılmış bir mekanda Moria'yı hareket ettirmek.
	- Moria'ya Pick & Place uygulaması yaptırmak.
	- Intel Realsense D435 kamerayı kullanarak "Gesture Prediction" yapmak.
	- Dynamic Window Approach vb. bir hareket planlama algoritması ile haritası çıkartılmış bir ortamda dinamik engellerden 	  kaçınma.
	- LİDAR sensörünü tamamen robottan çıkartmak ve tüm haritalama vb. işlemleri yalnızca Intel Realsense D435 kamera ile 	  	  gerçekleştirmek.
