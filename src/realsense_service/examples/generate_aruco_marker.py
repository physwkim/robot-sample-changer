#!/usr/bin/env python3
"""
ArUco 마커 생성 스크립트

Hand-Eye Calibration에 사용할 ArUco 마커를 PDF로 생성합니다.
"""

import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import argparse


def generate_aruco_marker(marker_id=0, marker_size_mm=50, dictionary_type='DICT_6X6_250',
                          output_file='aruco_marker.pdf', border_bits=1):
    """
    ArUco 마커 생성 및 PDF로 저장

    Args:
        marker_id (int): 마커 ID (0-249 for DICT_6X6_250)
        marker_size_mm (int): 마커 크기 (mm 단위, 검은색 영역만)
        dictionary_type (str): ArUco dictionary 타입
        output_file (str): 출력 PDF 파일명
        border_bits (int): 마커 테두리 비트 수 (기본 1)
    """

    # ArUco dictionary 가져오기
    aruco_dict_map = {
        'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
        'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
        'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
        'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
        'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
        'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
        'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
        'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
        'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
        'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
        'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
        'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
        'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
        'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
    }

    if dictionary_type not in aruco_dict_map:
        print(f"Error: 지원하지 않는 dictionary 타입입니다: {dictionary_type}")
        print(f"사용 가능한 타입: {list(aruco_dict_map.keys())}")
        return False

    # ArUco dictionary 로드
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[dictionary_type])

    # 마커 이미지 크기 (픽셀)
    # 고해상도로 생성 (1mm = 10 pixels)
    pixels_per_mm = 10
    marker_size_pixels = marker_size_mm * pixels_per_mm

    # 마커 생성 (검은색 영역만)
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_pixels, borderBits=border_bits)

    # White border 추가 (마커 크기의 25%)
    border_size = int(round(marker_size_pixels * 0.25))
    marker_with_border = cv2.copyMakeBorder(
        marker_image,
        border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT,
        value=255
    )

    # 전체 크기 계산
    total_size_mm = marker_size_mm + (border_size * 2 / pixels_per_mm)

    # DPI 설정 (1 inch = 25.4 mm)
    # 픽셀-물리 크기 매칭을 위해 pixels_per_mm 기준으로 설정
    dpi = pixels_per_mm * 25.4
    mm_per_inch = 25.4

    # Figure 크기를 실제 물리적 크기에 맞춤 (인치 단위)
    # 마커 주변에 여백 및 하단 정보/눈금 공간 추가
    page_margin_mm = 20
    top_margin_mm = page_margin_mm
    bottom_margin_mm = page_margin_mm
    ruler_height_mm = 6
    ruler_gap_mm = 6
    info_block_height_mm = 45
    page_width_mm = total_size_mm + page_margin_mm * 2
    page_height_mm = (
        top_margin_mm
        + total_size_mm
        + ruler_gap_mm
        + ruler_height_mm
        + ruler_gap_mm
        + info_block_height_mm
        + bottom_margin_mm
    )

    fig_width_inch = page_width_mm / mm_per_inch
    fig_height_inch = page_height_mm / mm_per_inch

    # PDF로 저장
    fig = plt.figure(figsize=(fig_width_inch, fig_height_inch), dpi=dpi)

    # 마커 표시 영역 (물리 크기 고정)
    marker_left_mm = page_margin_mm
    marker_bottom_mm = page_height_mm - top_margin_mm - total_size_mm
    ax = fig.add_axes([
        marker_left_mm / page_width_mm,
        marker_bottom_mm / page_height_mm,
        total_size_mm / page_width_mm,
        total_size_mm / page_height_mm,
    ])
    ax.imshow(marker_with_border, cmap='gray', interpolation='nearest')
    ax.set_aspect('equal')
    ax.axis('off')

    # 눈금자 추가 (크기 검증용)
    ruler_left_mm = page_margin_mm
    ruler_bottom_mm = marker_bottom_mm - ruler_gap_mm - ruler_height_mm
    ruler_ax = fig.add_axes([
        ruler_left_mm / page_width_mm,
        ruler_bottom_mm / page_height_mm,
        total_size_mm / page_width_mm,
        ruler_height_mm / page_height_mm,
    ])
    ruler_ax.set_xlim(0, total_size_mm)
    ruler_ax.set_ylim(0, 1)

    # 10mm 간격으로 눈금 표시
    for i in range(0, int(total_size_mm) + 1, 10):
        ruler_ax.axvline(x=i, color='black', linewidth=1)
        ruler_ax.text(i, 0.5, f'{i}mm', ha='center', va='center', fontsize=8)

    # 마커 영역 강조 (검은색 영역)
    marker_start = (total_size_mm - marker_size_mm) / 2
    marker_end = marker_start + marker_size_mm
    ruler_ax.axvspan(marker_start, marker_end, alpha=0.3, color='red')
    ruler_ax.text((marker_start + marker_end) / 2, -0.5,
                  f'Marker: {marker_size_mm}mm',
                  ha='center', va='top', fontsize=9, weight='bold', color='red')

    ruler_ax.set_xticks([])
    ruler_ax.set_yticks([])
    ruler_ax.spines['top'].set_visible(False)
    ruler_ax.spines['right'].set_visible(False)
    ruler_ax.spines['left'].set_visible(False)

    # 제목 및 정보 추가
    info_text = f"""ArUco Marker - Dictionary: {dictionary_type} | ID: {marker_id}
Marker Size (black area): {marker_size_mm} mm | Total Size (with border): {total_size_mm:.1f} mm

인쇄 방법: PDF 뷰어 설정 → "실제 크기" 또는 "100% 배율" 선택
검증 방법: 인쇄 후 자로 측정 → 위 눈금자와 비교 (검은색 영역 = {marker_size_mm}mm)
"""

    info_center_y_mm = bottom_margin_mm + (info_block_height_mm / 2)
    fig.text(0.5, info_center_y_mm / page_height_mm, info_text, ha='center', va='center',
             fontsize=9, family='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # PDF 저장 (DPI 명시)
    with PdfPages(output_file) as pdf:
        pdf.savefig(fig, dpi=dpi)
        plt.close()

    print("=" * 60)
    print("ArUco 마커가 성공적으로 생성되었습니다!")
    print("=" * 60)
    print(f"출력 파일: {output_file}")
    print(f"Dictionary: {dictionary_type}")
    print(f"Marker ID: {marker_id}")
    print(f"마커 크기 (검은색 영역): {marker_size_mm} mm")
    print(f"전체 크기 (흰색 테두리 포함): {total_size_mm:.1f} mm")
    print()
    print("인쇄 방법:")
    print("  1. PDF 뷰어에서 '실제 크기' 또는 '100% 배율'로 인쇄")
    print("  2. 인쇄 후 자로 마커 크기를 확인하세요")
    print(f"     → 검은색 영역이 정확히 {marker_size_mm} mm여야 합니다")
    print("  3. 가위로 자르고 평평한 표면에 부착")
    print()
    print("캘리브레이션 시 사용:")
    print(f"  marker_type: aruco")
    print(f"  marker_size: {marker_size_mm / 1000:.3f}  # meter 단위")
    print(f"  aruco_dict: {dictionary_type}")
    print("=" * 60)

    return True


def generate_multiple_markers(start_id=0, count=4, marker_size_mm=50,
                              dictionary_type='DICT_6X6_250',
                              output_file='aruco_markers_sheet.pdf'):
    """
    여러 개의 ArUco 마커를 한 장에 생성

    Args:
        start_id (int): 시작 마커 ID
        count (int): 생성할 마커 개수 (1-4)
        marker_size_mm (int): 각 마커 크기 (mm)
        dictionary_type (str): ArUco dictionary 타입
        output_file (str): 출력 PDF 파일명
    """

    if count > 4:
        print("Warning: 최대 4개까지 생성 가능합니다. 4개로 제한합니다.")
        count = 4

    aruco_dict_map = {
        'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    }

    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map.get(dictionary_type, cv2.aruco.DICT_6X6_250))
    pixels_per_mm = 10
    marker_size_pixels = marker_size_mm * pixels_per_mm
    dpi = pixels_per_mm * 25.4
    mm_per_inch = 25.4

    # 2x2 그리드로 배치 (물리 크기 고정)
    border_size = int(round(marker_size_pixels * 0.25))
    total_size_mm = marker_size_mm + (border_size * 2 / pixels_per_mm)
    page_margin_mm = 15
    gap_mm = 10
    info_block_height_mm = 15
    page_width_mm = total_size_mm * 2 + page_margin_mm * 2 + gap_mm
    page_height_mm = total_size_mm * 2 + page_margin_mm * 2 + gap_mm + info_block_height_mm
    fig_width_inch = page_width_mm / mm_per_inch
    fig_height_inch = page_height_mm / mm_per_inch
    fig = plt.figure(figsize=(fig_width_inch, fig_height_inch), dpi=dpi)

    axes = []
    left_col_mm = page_margin_mm
    right_col_mm = page_margin_mm + total_size_mm + gap_mm
    bottom_row_mm = page_margin_mm + info_block_height_mm
    top_row_mm = bottom_row_mm + total_size_mm + gap_mm
    positions_mm = [
        (left_col_mm, top_row_mm),
        (right_col_mm, top_row_mm),
        (left_col_mm, bottom_row_mm),
        (right_col_mm, bottom_row_mm),
    ]

    for left_mm, bottom_mm in positions_mm:
        axes.append(fig.add_axes([
            left_mm / page_width_mm,
            bottom_mm / page_height_mm,
            total_size_mm / page_width_mm,
            total_size_mm / page_height_mm,
        ]))

    for i in range(4):
        if i < count:
            marker_id = start_id + i
            marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_pixels, borderBits=1)
            border_size = int(round(marker_size_pixels * 0.25))
            marker_with_border = cv2.copyMakeBorder(
                marker_image, border_size, border_size, border_size, border_size,
                cv2.BORDER_CONSTANT, value=255
            )

            axes[i].imshow(marker_with_border, cmap='gray', interpolation='nearest')
            axes[i].set_aspect('equal')
            axes[i].text(
                0.5, 1.02, f'ID: {marker_id} | {marker_size_mm}mm',
                ha='center', va='bottom', transform=axes[i].transAxes,
                fontsize=10, weight='bold'
            )
        else:
            axes[i].text(0.5, 0.5, 'Empty', ha='center', va='center', transform=axes[i].transAxes)

        axes[i].axis('off')

    info_text = f"Dictionary: {dictionary_type} | Size: {marker_size_mm}mm | IDs: {start_id}-{start_id+count-1}"
    fig.text(0.5, (page_margin_mm / 2) / page_height_mm, info_text,
             ha='center', va='center', fontsize=10)

    with PdfPages(output_file) as pdf:
        pdf.savefig(fig, dpi=dpi)
        plt.close()

    print(f"\n{count}개의 마커가 생성되었습니다: {output_file}")
    print(f"Marker IDs: {start_id} ~ {start_id + count - 1}")


def main():
    parser = argparse.ArgumentParser(description='ArUco 마커 생성기')
    parser.add_argument('--id', type=int, default=0, help='마커 ID (기본: 0)')
    parser.add_argument('--size', type=int, default=50, help='마커 크기 mm (기본: 50)')
    parser.add_argument('--dict', type=str, default='DICT_6X6_250',
                       help='ArUco dictionary (기본: DICT_6X6_250)')
    parser.add_argument('--output', type=str, default='aruco_marker.pdf',
                       help='출력 파일명 (기본: aruco_marker.pdf)')
    parser.add_argument('--multiple', action='store_true',
                       help='여러 개 마커 생성 (2x2 그리드)')
    parser.add_argument('--count', type=int, default=4,
                       help='생성할 마커 개수 (--multiple 사용 시, 기본: 4)')

    args = parser.parse_args()

    if args.multiple:
        generate_multiple_markers(
            start_id=args.id,
            count=args.count,
            marker_size_mm=args.size,
            dictionary_type=args.dict,
            output_file=args.output
        )
    else:
        generate_aruco_marker(
            marker_id=args.id,
            marker_size_mm=args.size,
            dictionary_type=args.dict,
            output_file=args.output
        )


if __name__ == '__main__':
    main()
