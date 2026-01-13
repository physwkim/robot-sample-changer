#!/usr/bin/env python3
"""
카메라로부터 Planning Scene을 주기적으로 업데이트하는 예제

이 스크립트는 RealSense 카메라의 depth 데이터를 사용하여
MoveIt의 Planning Scene에 장애물을 추가합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from std_msgs.msg import Header
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
from sklearn.cluster import DBSCAN
try:
    from scipy.spatial import ConvexHull
    _HAS_SCIPY = True
except Exception:
    ConvexHull = None
    _HAS_SCIPY = False


class PlanningSceneUpdater(Node):
    def __init__(self):
        super().__init__('planning_scene_updater')

        # CV Bridge
        self.bridge = CvBridge()

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/realsense_service_node/depth/image_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/realsense_service_node/depth/camera_info',
            self.camera_info_callback,
            10
        )

        # Parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('min_depth_threshold', 0.18)  # meters (18cm 이내 제외, gripper 필터링)
        self.declare_parameter('depth_threshold', 0.5)  # meters (50cm 이상 제외)
        self.declare_parameter('min_obstacle_points', 100)
        self.declare_parameter('clustering_eps', 0.05)  # DBSCAN epsilon (5cm)
        self.declare_parameter('clustering_min_samples', 10)  # DBSCAN min samples

        update_rate = self.get_parameter('update_rate').value

        # Camera intrinsic parameters (RealSense D405 @ 848x480 - actual calibrated values)
        self.camera_fx = 431.73626709  # focal length x (from depth camera calibration)
        self.camera_fy = 431.73626709  # focal length y (from depth camera calibration)
        self.camera_cx = 427.109375    # principal point x (from depth camera calibration)
        self.camera_cy = 244.54608154  # principal point y (from depth camera calibration)
        self.camera_frame = 'camera_link_depth_optical_frame'
        self.camera_link_frame = 'camera_link'
        self.camera_info_ready = False

        # Timer for periodic updates
        self.timer = self.create_timer(1.0 / update_rate, self.update_scene)

        # Latest depth image
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.latest_depth_frame = None
        self.logged_depth_frame = False
        self.logged_tf_frames = False

        self.get_logger().info('Planning Scene Updater 시작')
        self.get_logger().info(f'업데이트 주기: {update_rate} Hz')
        self.get_logger().info(f'Depth 범위: {self.get_parameter("min_depth_threshold").value}m ~ {self.get_parameter("depth_threshold").value}m')

    def depth_callback(self, msg):
        """Depth 이미지 수신"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            self.latest_depth_stamp = msg.header.stamp
            self.latest_depth_frame = msg.header.frame_id
            if self.latest_depth_frame and not self.logged_depth_frame:
                self.get_logger().info(f'Depth frame_id: {self.latest_depth_frame}')
                self.logged_depth_frame = True
        except Exception as e:
            self.get_logger().error(f'Depth 이미지 변환 실패: {e}')

    def camera_info_callback(self, msg):
        """CameraInfo로 Intrinsic 업데이트"""
        if len(msg.k) >= 9:
            self.camera_fx = msg.k[0]
            self.camera_fy = msg.k[4]
            self.camera_cx = msg.k[2]
            self.camera_cy = msg.k[5]
            if msg.header.frame_id:
                self.camera_frame = msg.header.frame_id
                if self.camera_frame.endswith('_depth_optical_frame'):
                    self.camera_link_frame = self.camera_frame.replace('_depth_optical_frame', '')
                elif self.camera_frame.endswith('_color_optical_frame'):
                    self.camera_link_frame = self.camera_frame.replace('_color_optical_frame', '')
            if not self.camera_info_ready:
                self.get_logger().info(
                    f'CameraInfo 수신: fx={self.camera_fx:.3f}, fy={self.camera_fy:.3f}, '
                    f'cx={self.camera_cx:.3f}, cy={self.camera_cy:.3f}, frame={self.camera_frame}'
                )
            self.camera_info_ready = True

    def update_scene(self):
        """Planning Scene 업데이트"""
        if self.latest_depth is None:
            self.get_logger().warn('Depth 이미지가 아직 수신되지 않았습니다')
            return

        try:
            # Planning Scene 메시지 생성
            scene = PlanningScene()
            scene.is_diff = True  # Incremental update
            scene.world.collision_objects = []

            # Depth 이미지에서 장애물 감지 및 추가
            # (실제로는 point cloud processing 필요)
            obstacles = self.detect_obstacles_from_depth(self.latest_depth)
            scene.world.collision_objects.extend(obstacles)

            # Publish
            self.scene_pub.publish(scene)

            if len(obstacles) > 0:
                self.get_logger().info(f'Planning Scene 업데이트: {len(obstacles)}개 장애물')

        except Exception as e:
            self.get_logger().error(f'Scene 업데이트 실패: {e}')


    def depth_to_point_cloud(self, depth_image):
        """
        Depth 이미지를 3D Point Cloud로 변환

        Args:
            depth_image: numpy array (H x W), 16UC1 format, mm 단위

        Returns:
            numpy array (N x 3), 각 행은 [x, y, z] 좌표 (meters)
        """
        h, w = depth_image.shape
        points = []

        # Depth threshold (0.1mm 단위 → meters)
        # RealSense D405는 depth를 0.1mm 단위로 인코딩
        min_depth_threshold = self.get_parameter('min_depth_threshold').value * 10000.0
        depth_threshold = self.get_parameter('depth_threshold').value * 10000.0

        for v in range(0, h, 4):  # Downsample: 매 4픽셀마다
            for u in range(0, w, 4):
                z = depth_image[v, u]

                # 유효하지 않은 depth 값 제외 (너무 가까우면 gripper, 너무 멀면 관심 밖)
                if z == 0 or z < min_depth_threshold or z > depth_threshold:
                    continue

                # Pixel → 3D 좌표 변환 (Pinhole camera model)
                z_m = z / 10000.0  # 0.1mm → meters
                x = (u - self.camera_cx) * z_m / self.camera_fx
                y = (v - self.camera_cy) * z_m / self.camera_fy

                points.append([x, y, z_m])

        points_array = np.array(points) if len(points) > 0 else np.empty((0, 3))

        if len(points_array) > 0:
            self.get_logger().debug(f'Point cloud: {len(points_array)} points')
            self.get_logger().debug(f'  X range: [{points_array[:, 0].min():.3f}, {points_array[:, 0].max():.3f}]')
            self.get_logger().debug(f'  Y range: [{points_array[:, 1].min():.3f}, {points_array[:, 1].max():.3f}]')
            self.get_logger().debug(f'  Z range: [{points_array[:, 2].min():.3f}, {points_array[:, 2].max():.3f}]')

        return points_array

    def cluster_point_cloud(self, points):
        """
        Point Cloud를 DBSCAN으로 클러스터링

        Args:
            points: numpy array (N x 3)

        Returns:
            list of numpy arrays, 각 클러스터별 포인트
        """
        if len(points) < 10:
            return []

        eps = self.get_parameter('clustering_eps').value
        min_samples = self.get_parameter('clustering_min_samples').value

        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = clustering.labels_

        # 각 클러스터별로 포인트 그룹화 (noise 제외: label != -1)
        clusters = []
        unique_labels = set(labels)
        num_noise = np.sum(labels == -1)

        for label in unique_labels:
            if label == -1:  # Noise
                continue
            cluster_points = points[labels == label]
            clusters.append(cluster_points)

        self.get_logger().info(f'Clustering: {len(clusters)} clusters found, {num_noise} noise points')

        return clusters

    def split_clusters_by_height(self, clusters, height_threshold=0.03):
        """
        각 클러스터를 높이(Z) 차이 기준으로 추가 분리

        Args:
            clusters: list of numpy arrays
            height_threshold: 높이 차이 임계값 (meters, 기본 3cm)

        Returns:
            list of numpy arrays (분리된 클러스터)
        """
        split_clusters = []

        for cluster in clusters:
            if len(cluster) < 10:
                split_clusters.append(cluster)
                continue

            # Z축(높이) 기준으로 재클러스터링
            z_coords = cluster[:, 2].reshape(-1, 1)
            z_clustering = DBSCAN(eps=height_threshold, min_samples=5).fit(z_coords)
            z_labels = z_clustering.labels_

            # 각 Z 레이어별로 분리
            unique_z_labels = set(z_labels)
            for z_label in unique_z_labels:
                if z_label == -1:  # Noise
                    continue
                sub_cluster = cluster[z_labels == z_label]
                if len(sub_cluster) >= 10:
                    split_clusters.append(sub_cluster)

        self.get_logger().info(f'Height-based split: {len(clusters)} → {len(split_clusters)} clusters')

        return split_clusters

    def transform_point_to_base_link(self, point_camera):
        """
        Camera frame의 좌표를 base_link로 변환

        Args:
            point_camera: numpy array [x, y, z] in camera frame

        Returns:
            numpy array [x, y, z] in base_link frame, or None if transform fails
        """
        try:
            # PointStamped 메시지 생성
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = float(point_camera[0])
            point_stamped.point.y = float(point_camera[1])
            point_stamped.point.z = float(point_camera[2])

            # TF 변환
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # 변환 적용
            point_base = tf2_geometry_msgs.do_transform_point(point_stamped, transform)

            return np.array([point_base.point.x, point_base.point.y, point_base.point.z])

        except (TransformException, Exception) as e:
            self.get_logger().error(f'TF 변환 실패: {e}')
            return None

    def lookup_transform(self, target_frame, source_frame):
        """source_frame -> target_frame 변환을 조회"""
        # 최신 사용 가능한 변환 사용 (time=0)
        time = rclpy.time.Time()

        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except (TransformException, Exception) as e:
            self.get_logger().warn(
                f'TF 변환 조회 실패 (source="{source_frame}" -> target="{target_frame}"): {e}'
            )
            return None

    def transform_points(self, points_src, transform):
        """source frame의 점들을 target frame으로 변환"""
        t = transform.transform.translation
        q = transform.transform.rotation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        rotation = np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)]
        ])
        translation = np.array([t.x, t.y, t.z])

        return (points_src @ rotation.T) + translation

    def cluster_to_bounding_box(self, cluster_points, cluster_id, transform):
        """
        클러스터를 Axis-Aligned Bounding Box (AABB)로 변환

        Args:
            cluster_points: numpy array (N x 3) in camera frame
            cluster_id: 클러스터 식별자
            transform: camera -> base_link 변환

        Returns:
            CollisionObject or None
        """
        min_points = self.get_parameter('min_obstacle_points').value
        if len(cluster_points) < min_points:
            self.get_logger().debug(f'Cluster {cluster_id}: {len(cluster_points)} points < {min_points} (skipped)')
            return None

        # Bounding Box 계산 (camera frame에서)
        min_coords = np.min(cluster_points, axis=0)
        max_coords = np.max(cluster_points, axis=0)

        # 중심점과 크기 (camera frame)
        center_camera = (min_coords + max_coords) / 2.0
        dimensions = max_coords - min_coords

        # 너무 작은 물체 제외 (1cm 이하)
        if np.any(dimensions < 0.01):
            self.get_logger().debug(f'Cluster {cluster_id}: dimensions {dimensions} too small (skipped)')
            return None

        # 클러스터 전체를 base_link frame으로 변환 후 AABB 계산
        points_base = self.transform_points(cluster_points, transform)
        min_base = np.min(points_base, axis=0)
        max_base = np.max(points_base, axis=0)
        center_base = (min_base + max_base) / 2.0
        dimensions_base = max_base - min_base

        if np.any(dimensions_base < 0.01):
            self.get_logger().debug(f'Cluster {cluster_id}: dimensions {dimensions_base} too small (skipped)')
            return None

        # 로그 출력
        self.get_logger().info(f'Cluster {cluster_id}: {len(cluster_points)} points')
        self.get_logger().info(f'  Center (camera): X={center_camera[0]:.3f}, Y={center_camera[1]:.3f}, Z={center_camera[2]:.3f}')
        self.get_logger().info(f'  Center (base_link): X={center_base[0]:.3f}, Y={center_base[1]:.3f}, Z={center_base[2]:.3f}')
        self.get_logger().info(f'  Size: {dimensions_base[0]:.3f} x {dimensions_base[1]:.3f} x {dimensions_base[2]:.3f} m')

        # CollisionObject 생성 (base_link frame으로)
        collision_object = CollisionObject()
        collision_object.header = Header()
        collision_object.header.frame_id = 'base_link'  # base_link frame 사용
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = f'obstacle_{cluster_id}'

        # Box primitive
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions_base.tolist()

        # Pose (base_link frame)
        pose = PoseStamped()
        pose.header = collision_object.header
        pose.pose.position.x = center_base[0]
        pose.pose.position.y = center_base[1]
        pose.pose.position.z = center_base[2]
        pose.pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose.pose)
        collision_object.operation = CollisionObject.ADD

        return collision_object

    def cluster_to_mesh(self, cluster_points, cluster_id, transform):
        """
        클러스터를 Convex Hull Mesh로 변환

        Args:
            cluster_points: numpy array (N x 3) in camera frame
            cluster_id: 클러스터 식별자
            transform: camera -> base_link 변환

        Returns:
            CollisionObject or None
        """
        min_points = self.get_parameter('min_obstacle_points').value
        if len(cluster_points) < min_points:
            self.get_logger().debug(f'Cluster {cluster_id}: {len(cluster_points)} points < {min_points} (skipped)')
            return None

        if not _HAS_SCIPY:
            self.get_logger().warn('scipy가 없어 mesh 생성이 불가합니다. bounding box로 대체합니다.')
            return self.cluster_to_bounding_box(cluster_points, cluster_id, transform)

        # base_link frame으로 변환
        points_base = self.transform_points(cluster_points, transform)
        if len(points_base) < 4:
            self.get_logger().debug(f'Cluster {cluster_id}: insufficient points for mesh (skipped)')
            return None

        # Convex Hull 계산
        try:
            hull = ConvexHull(points_base)
        except Exception as e:
            self.get_logger().warn(f'Cluster {cluster_id}: ConvexHull 실패: {e}')
            return None

        # Mesh 생성
        mesh = Mesh()
        mesh.vertices = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points_base]

        for simplex in hull.simplices:
            tri = MeshTriangle()
            tri.vertex_indices = [int(simplex[0]), int(simplex[1]), int(simplex[2])]
            mesh.triangles.append(tri)

        # CollisionObject 생성 (base_link frame)
        collision_object = CollisionObject()
        collision_object.header = Header()
        collision_object.header.frame_id = 'base_link'
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = f'obstacle_{cluster_id}'

        pose = PoseStamped()
        pose.header = collision_object.header
        pose.pose.orientation.w = 1.0

        collision_object.meshes.append(mesh)
        collision_object.mesh_poses.append(pose.pose)
        collision_object.operation = CollisionObject.ADD

        self.get_logger().info(f'Cluster {cluster_id}: mesh with {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles')

        return collision_object

    def detect_obstacles_from_depth(self, depth_image):
        """
        Depth 이미지로부터 장애물 감지

        실제 구현:
        1. Depth → Point Cloud 변환
        2. Clustering (DBSCAN)
        3. 각 클러스터를 Mesh로 근사
        4. CollisionObject 생성
        """
        obstacles = []

        # 1. Depth → Point Cloud 변환
        points = self.depth_to_point_cloud(depth_image)
        if len(points) == 0:
            return obstacles

        # depth frame -> camera_link 변환 (optical frame인 경우 정렬)
        source_frame = self.latest_depth_frame or self.camera_frame
        if source_frame != self.camera_link_frame:
            to_camera_link = self.lookup_transform(self.camera_link_frame, source_frame)
            if to_camera_link is None:
                return obstacles
            points = self.transform_points(points, to_camera_link)
            source_frame = self.camera_link_frame

        # 2. Clustering (camera_link 기준)
        clusters = self.cluster_point_cloud(points)

        # 2-1. 높이 차이 기준으로 추가 분리 (20mm 이상 차이나는 층 분리)
        clusters = self.split_clusters_by_height(clusters, height_threshold=0.02)

        # camera_link -> base_link 변환
        transform = self.lookup_transform('base_link', source_frame)
        if transform is None:
            return obstacles

        # 3. 각 클러스터를 Bounding Box로 변환 (높이별로 분리된 직사각형)
        for i, cluster in enumerate(clusters):
            bbox = self.cluster_to_bounding_box(cluster, i, transform)
            if bbox is not None:
                obstacles.append(bbox)

        return obstacles


def main(args=None):
    rclpy.init(args=args)

    updater = PlanningSceneUpdater()

    try:
        rclpy.spin(updater)
    except KeyboardInterrupt:
        pass
    finally:
        updater.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
