#!/usr/bin/env python3
"""
카메라 intrinsic 파라미터 테스트 예제

이 스크립트는 RealSense 카메라의 intrinsic 파라미터를
자동으로 가져와서 매트릭스로 구성하는 방법을 보여줍니다.
"""

import pyrealsense2 as rs
import numpy as np


def get_camera_intrinsics():
    """RealSense 카메라의 intrinsic 파라미터 가져오기"""

    # 파이프라인 설정
    pipeline = rs.pipeline()
    config = rs.config()

    # 스트림 설정 (640x480 또는 848x480)
    width, height, fps = 848, 480, 30
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    # 파이프라인 시작
    profile = pipeline.start(config)

    try:
        # Depth 스트림 intrinsic 파라미터
        stream_depth = profile.get_stream(rs.stream.depth)
        intrinsic_depth = stream_depth.as_video_stream_profile().get_intrinsics()

        # Color 스트림 intrinsic 파라미터
        stream_color = profile.get_stream(rs.stream.color)
        intrinsic_color = stream_color.as_video_stream_profile().get_intrinsics()

        # Depth scale 가져오기
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        # Depth 카메라 파라미터 및 매트릭스 구성
        cam_params_depth = [
            intrinsic_depth.fx,
            intrinsic_depth.fy,
            intrinsic_depth.ppx,
            intrinsic_depth.ppy
        ]
        cam_matrix_depth = np.array([
            [intrinsic_depth.fx, 0, intrinsic_depth.ppx],
            [0, intrinsic_depth.fy, intrinsic_depth.ppy],
            [0, 0, 1]
        ])

        # Color 카메라 파라미터 및 매트릭스 구성
        cam_params_color = [
            intrinsic_color.fx,
            intrinsic_color.fy,
            intrinsic_color.ppx,
            intrinsic_color.ppy
        ]
        cam_matrix_color = np.array([
            [intrinsic_color.fx, 0, intrinsic_color.ppx],
            [0, intrinsic_color.fy, intrinsic_color.ppy],
            [0, 0, 1]
        ])

        # 결과 출력
        print("=" * 60)
        print("RealSense Camera Intrinsic Parameters")
        print("=" * 60)

        print("\n[Depth Scale]")
        print(f"  {depth_scale} (meter per unit)")

        print("\n[Color Camera Parameters]")
        print(f"  fx: {cam_params_color[0]:.2f}")
        print(f"  fy: {cam_params_color[1]:.2f}")
        print(f"  ppx: {cam_params_color[2]:.2f}")
        print(f"  ppy: {cam_params_color[3]:.2f}")

        print("\n[Color Camera Matrix]")
        print(cam_matrix_color)

        print("\n[Depth Camera Parameters]")
        print(f"  fx: {cam_params_depth[0]:.2f}")
        print(f"  fy: {cam_params_depth[1]:.2f}")
        print(f"  ppx: {cam_params_depth[2]:.2f}")
        print(f"  ppy: {cam_params_depth[3]:.2f}")

        print("\n[Depth Camera Matrix]")
        print(cam_matrix_depth)

        print("\n" + "=" * 60)

        return {
            'depth_scale': depth_scale,
            'cam_params_color': cam_params_color,
            'cam_matrix_color': cam_matrix_color,
            'cam_params_depth': cam_params_depth,
            'cam_matrix_depth': cam_matrix_depth
        }

    finally:
        # 파이프라인 정지
        pipeline.stop()


if __name__ == '__main__':
    try:
        intrinsics = get_camera_intrinsics()
        print("\n카메라 intrinsic 파라미터를 성공적으로 가져왔습니다!")
    except Exception as e:
        print(f"\n에러 발생: {str(e)}")
