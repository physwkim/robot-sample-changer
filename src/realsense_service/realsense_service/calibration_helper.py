#!/usr/bin/env python3
"""
Hand-Eye Calibration Helper Script

사용자가 로봇을 여러 포즈로 이동시키면서 캘리브레이션 데이터를 수집하는 헬퍼 스크립트
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys
import termios
import tty


class CalibrationHelper(Node):
    def __init__(self):
        super().__init__('calibration_helper')

        # 서비스 클라이언트 생성
        self.capture_client = self.create_client(Trigger, 'capture_calibration_sample')
        self.compute_client = self.create_client(Trigger, 'compute_calibration')
        self.reset_client = self.create_client(Trigger, 'reset_calibration')

        # 서비스 대기
        self.get_logger().info('캘리브레이션 서비스 대기 중...')
        self.capture_client.wait_for_service(timeout_sec=5.0)
        self.compute_client.wait_for_service(timeout_sec=5.0)
        self.reset_client.wait_for_service(timeout_sec=5.0)

        self.sample_count = 0

    def capture_sample(self):
        """샘플 캡처"""
        request = Trigger.Request()
        future = self.capture_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.sample_count += 1
                print(f'\n✓ {response.message}')
                return True
            else:
                print(f'\n✗ 실패: {response.message}')
                return False
        else:
            print('\n✗ 서비스 호출 실패')
            return False

    def compute_calibration(self):
        """캘리브레이션 계산"""
        print('\n캘리브레이션 계산 중...')
        request = Trigger.Request()
        future = self.compute_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                print(f'\n✓ {response.message}')
                return True
            else:
                print(f'\n✗ 실패: {response.message}')
                return False
        else:
            print('\n✗ 서비스 호출 실패')
            return False

    def reset_calibration(self):
        """캘리브레이션 초기화"""
        request = Trigger.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            print(f'\n{response.message}')
            self.sample_count = 0
            return True
        else:
            print('\n✗ 서비스 호출 실패')
            return False


def get_key():
    """키 입력 받기"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def print_instructions():
    """사용 안내 출력"""
    print('\n' + '='*60)
    print('Hand-Eye Calibration Helper')
    print('='*60)
    print('\n사용 방법:')
    print('  1. 로봇을 다양한 포즈로 이동시킵니다')
    print('  2. ArUco 마커/체커보드가 카메라에 잘 보이는지 확인합니다')
    print('  3. SPACE 키를 눌러 샘플을 캡처합니다')
    print('  4. 최소 10개의 샘플을 수집합니다')
    print('  5. 충분한 샘플이 모이면 C 키를 눌러 캘리브레이션을 계산합니다')
    print('\n키 명령:')
    print('  SPACE : 현재 포즈에서 샘플 캡처')
    print('  C     : 캘리브레이션 계산')
    print('  R     : 데이터 초기화')
    print('  Q     : 종료')
    print('='*60)
    print('\n팁:')
    print('  - 로봇을 다양한 각도와 위치로 이동시키세요')
    print('  - 각 포즈에서 마커가 명확하게 보이는지 확인하세요')
    print('  - 로봇의 작업 공간 전체에 걸쳐 샘플을 수집하세요')
    print('  - 15-20개의 샘플을 권장합니다')
    print('='*60 + '\n')


def main(args=None):
    rclpy.init(args=args)
    helper = CalibrationHelper()

    print_instructions()

    try:
        while rclpy.ok():
            print(f'\n명령 대기 중... (현재 샘플 수: {helper.sample_count})')
            print('> ', end='', flush=True)

            key = get_key()

            if key == ' ':
                print('SPACE - 샘플 캡처')
                helper.capture_sample()

            elif key.lower() == 'c':
                print('C - 캘리브레이션 계산')
                if helper.compute_calibration():
                    print('\n캘리브레이션이 완료되었습니다!')
                    print('결과 파일이 ~/calibration_data 에 저장되었습니다.')
                    break

            elif key.lower() == 'r':
                print('R - 초기화')
                helper.reset_calibration()

            elif key.lower() == 'q':
                print('Q - 종료')
                break

            else:
                print(f'알 수 없는 명령: {key}')

    except KeyboardInterrupt:
        print('\n\n종료 중...')

    finally:
        helper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
