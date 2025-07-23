#!/usr/bin/env python3
"""
追跡統合制御モジュールのテスト

TrackingCoordinatorクラスの各機能をテストし、
統合システムが正しく動作することを確認します。

テスト項目:
- 初期化テスト
- モック環境での動作テスト
- システム状態管理テスト
- エラーハンドリングテスト
"""

import unittest
import sys
import os
import logging
import time
import threading
from unittest.mock import patch, MagicMock, Mock
import numpy as np
from datetime import datetime

# プロジェクトルートをパスに追加
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

# ハードウェアライブラリをモック（テスト環境用）
sys.modules['board'] = Mock()
sys.modules['busio'] = Mock()
sys.modules['adafruit_pca9685'] = Mock()
sys.modules['adafruit_motor'] = Mock()
sys.modules['adafruit_motor.servo'] = Mock()
sys.modules['ultralytics'] = Mock()
sys.modules['cv2'] = Mock()

# OpenCV用の基本的なモック設定
mock_cv2 = Mock()
mock_cv2.VideoCapture = Mock()
mock_cv2.CAP_PROP_FRAME_WIDTH = 3
mock_cv2.CAP_PROP_FRAME_HEIGHT = 4
mock_cv2.CAP_PROP_FPS = 5
mock_cv2.FONT_HERSHEY_SIMPLEX = 1
mock_cv2.waitKey = Mock(return_value=255)
mock_cv2.destroyAllWindows = Mock()
mock_cv2.imshow = Mock()
mock_cv2.rectangle = Mock()
mock_cv2.putText = Mock()
mock_cv2.circle = Mock()
mock_cv2.line = Mock()
sys.modules['cv2'] = mock_cv2

from modules.tracking_coordinator import (
    TrackingCoordinator,
    TrackingMode,
    SystemStatus
)


class TestTrackingCoordinator(unittest.TestCase):
    """TrackingCoordinatorクラスのテスト"""
    
    def setUp(self):
        """テスト前の準備"""
        # ログレベルを警告以上に設定（テスト中のログ出力を抑制）
        logging.getLogger().setLevel(logging.WARNING)
        
        # モック環境でコーディネーター作成
        with patch('cv2.VideoCapture'), \
             patch('modules.tracking_coordinator.ServoController'), \
             patch('modules.tracking_coordinator.YOLODetector'), \
             patch('modules.tracking_coordinator.SimpleProportionalController'):
            
            self.coordinator = TrackingCoordinator(
                camera_id=0,
                show_display=False  # テスト時は表示なし
            )
    
    def tearDown(self):
        """テスト後のクリーンアップ"""
        if hasattr(self.coordinator, 'is_running') and self.coordinator.is_running:
            self.coordinator.stop_tracking()
    
    def test_initialization(self):
        """初期化テスト"""
        # 基本設定の確認
        self.assertEqual(self.coordinator.camera_id, 0)
        self.assertEqual(self.coordinator.image_width, 640)
        self.assertEqual(self.coordinator.image_height, 480)
        self.assertFalse(self.coordinator.show_display)
        
        # 初期状態の確認
        self.assertEqual(self.coordinator.status.mode, TrackingMode.STANDBY)
        self.assertFalse(self.coordinator.status.target_detected)
        self.assertFalse(self.coordinator.is_running)
    
    def test_system_status(self):
        """システム状態テスト"""
        # 初期状態確認
        status = self.coordinator.get_system_status()
        
        self.assertEqual(status['mode'], 'standby')
        self.assertFalse(status['target_detected'])
        self.assertEqual(status['target_class'], '')
        self.assertEqual(status['target_confidence'], 0.0)
        self.assertEqual(status['pan_angle'], 0.0)
        self.assertEqual(status['tilt_angle'], 0.0)
        self.assertEqual(status['total_detections'], 0)
        self.assertFalse(status['is_running'])
    
    @patch('cv2.VideoCapture')
    @patch('modules.tracking_coordinator.ServoController')
    @patch('modules.tracking_coordinator.YOLODetector')
    @patch('modules.tracking_coordinator.SimpleProportionalController')
    def test_system_initialization(self, mock_simple_p, mock_yolo, mock_servo, mock_camera):
        """システム初期化テスト"""
        # モックの設定
        mock_camera_instance = Mock()
        mock_camera_instance.isOpened.return_value = True
        mock_camera.return_value = mock_camera_instance
        
        mock_servo_instance = Mock()
        mock_servo_instance.initialize.return_value = True
        mock_servo.return_value = mock_servo_instance
        
        mock_yolo_instance = Mock()
        mock_yolo_instance.load_model.return_value = True
        mock_yolo.return_value = mock_yolo_instance
        
        # 初期化テスト
        result = self.coordinator.initialize_system()
        
        # 結果確認
        self.assertTrue(result)
        self.assertEqual(self.coordinator.status.mode, TrackingMode.SCANNING)
        
        # モック呼び出し確認
        mock_camera.assert_called_once()
        mock_servo.assert_called_once()
        mock_yolo.assert_called_once()
        mock_simple_p.assert_called_once()
    
    @patch('cv2.VideoCapture')
    def test_camera_initialization_failure(self, mock_camera):
        """カメラ初期化失敗テスト"""
        # カメラが開けない場合をシミュレート
        mock_camera_instance = Mock()
        mock_camera_instance.isOpened.return_value = False
        mock_camera.return_value = mock_camera_instance
        
        # 初期化テスト
        result = self.coordinator.initialize_system()
        
        # 失敗することを確認
        self.assertFalse(result)
    
    def test_tracking_modes(self):
        """追跡モードテスト"""
        # モード変更テスト
        self.coordinator.status.mode = TrackingMode.SCANNING
        self.assertEqual(self.coordinator.status.mode, TrackingMode.SCANNING)
        
        self.coordinator.status.mode = TrackingMode.TRACKING
        self.assertEqual(self.coordinator.status.mode, TrackingMode.TRACKING)
        
        self.coordinator.status.mode = TrackingMode.STANDBY
        self.assertEqual(self.coordinator.status.mode, TrackingMode.STANDBY)
    
    def test_detection_processing(self):
        """検出処理テスト"""
        # モックフレーム作成
        mock_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # YOLODetectorをモック
        with patch.object(self.coordinator, 'yolo_detector') as mock_yolo:
            # 検出結果をモック
            mock_detection = {
                'class_id': 16,
                'class_name': 'Dog',
                'confidence': 0.85,
                'bbox': (100, 100, 200, 200)
            }
            mock_yolo.detect_pets.return_value = [mock_detection]
            mock_yolo.get_best_detection.return_value = mock_detection
            
            # 検出処理実行
            results = self.coordinator._process_detection(mock_frame)
            
            # 結果確認
            self.assertEqual(len(results), 1)
            self.assertEqual(results[0]['class_name'], 'Dog')
            self.assertEqual(results[0]['confidence'], 0.85)
    
    def test_tracking_control_with_detection(self):
        """検出ありの追跡制御テスト"""
        # 必要なモジュールをモック
        self.coordinator.servo_controller = Mock()
        self.coordinator.servo_controller.is_angle_safe.return_value = True
        self.coordinator.simple_p_controller = Mock()
        self.coordinator.simple_p_controller.calculate_correction.return_value = (2.0, -1.0)
        
        # 検出結果をシミュレート
        detections = [{
            'class_id': 15,
            'class_name': 'Cat',
            'confidence': 0.75,
            'bbox': (150, 120, 250, 220)
        }]
        
        # 追跡制御実行
        self.coordinator._update_tracking_control(detections)
        
        # 状態確認
        self.assertEqual(self.coordinator.status.mode, TrackingMode.TRACKING)
        self.assertTrue(self.coordinator.status.target_detected)
        self.assertEqual(self.coordinator.status.target_class, 'Cat')
        self.assertEqual(self.coordinator.status.target_confidence, 0.75)
        
        # サーボ制御呼び出し確認
        self.coordinator.servo_controller.set_angles.assert_called_once()
    
    def test_tracking_control_without_detection(self):
        """検出なしの追跡制御テスト"""
        # 事前に追跡モードに設定
        self.coordinator.status.mode = TrackingMode.TRACKING
        self.coordinator.status.last_detection_time = datetime.now()
        
        # 必要なモジュールをモック
        self.coordinator.servo_controller = Mock()
        
        # 検出なしの場合をシミュレート
        detections = []
        
        # 追跡制御実行
        self.coordinator._update_tracking_control(detections)
        
        # 状態確認
        self.assertFalse(self.coordinator.status.target_detected)
    
    def test_scan_pattern_execution(self):
        """スキャンパターン実行テスト"""
        # サーボコントローラーをモック
        self.coordinator.servo_controller = Mock()
        self.coordinator.servo_controller.is_angle_safe.return_value = True
        
        # スキャンパターン実行
        self.coordinator._execute_scan_pattern()
        
        # サーボ制御が呼び出されることを確認
        self.coordinator.servo_controller.set_angles.assert_called_once()
    
    def test_display_frame_creation(self):
        """表示フレーム作成テスト"""
        # テスト用フレーム作成
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 検出結果
        detections = [{
            'class_name': 'Dog',
            'confidence': 0.90,
            'bbox': (100, 100, 200, 200)
        }]
        
        # 表示フレーム作成
        display_frame = self.coordinator._create_display_frame(frame, detections)
        
        # フレームサイズが正しいことを確認
        self.assertEqual(display_frame.shape, (480, 640, 3))
    
    def test_system_status_update(self):
        """システム状態更新テスト"""
        # 初期状態確認
        self.assertEqual(self.coordinator.status.total_detections, 0)
        
        # 検出ありで更新
        detections = [{'class_name': 'Cat', 'confidence': 0.8}]
        self.coordinator._update_system_status(detections)
        
        # 検出回数が増加することを確認
        self.assertEqual(self.coordinator.status.total_detections, 1)
        
        # 検出なしで更新
        self.coordinator._update_system_status([])
        
        # 検出回数が変わらないことを確認
        self.assertEqual(self.coordinator.status.total_detections, 1)
    
    def test_error_handling_in_detection(self):
        """検出処理でのエラーハンドリングテスト"""
        # YOLODetectorでエラーが発生する場合をモック
        with patch.object(self.coordinator, 'yolo_detector') as mock_yolo:
            mock_yolo.detect_pets.side_effect = Exception("Detection error")
            
            # テスト用フレーム
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # エラーが発生しても空のリストが返されることを確認
            results = self.coordinator._process_detection(frame)
            self.assertEqual(results, [])
    
    def test_error_handling_in_tracking_control(self):
        """追跡制御でのエラーハンドリングテスト"""
        # サーボコントローラーでエラーが発生する場合をモック
        self.coordinator.servo_controller = Mock()
        self.coordinator.servo_controller.is_angle_safe.side_effect = Exception("Servo error")
        self.coordinator.simple_p_controller = Mock()
        
        detections = [{
            'class_name': 'Dog',
            'confidence': 0.8,
            'bbox': (100, 100, 200, 200)
        }]
        
        # エラーが発生しても例外が発生しないことを確認
        try:
            self.coordinator._update_tracking_control(detections)
        except Exception:
            self.fail("追跡制御でエラーハンドリングが失敗しました")
    
    def test_cleanup_resources(self):
        """リソース解放テスト"""
        # モックリソースを設定
        self.coordinator.camera = Mock()
        self.coordinator.servo_controller = Mock()
        self.coordinator.yolo_detector = Mock()
        self.coordinator.simple_p_controller = Mock()
        
        # リソース解放実行
        self.coordinator._cleanup_resources()
        
        # 各リソースのクリーンアップが呼び出されることを確認
        self.coordinator.camera.release.assert_called_once()
        self.coordinator.servo_controller.cleanup.assert_called_once()
        self.coordinator.yolo_detector.cleanup.assert_called_once()
        self.coordinator.simple_p_controller.cleanup.assert_called_once()


class TestSystemStatus(unittest.TestCase):
    """SystemStatusクラスのテスト"""
    
    def test_system_status_initialization(self):
        """システム状態初期化テスト"""
        status = SystemStatus()
        
        # デフォルト値確認
        self.assertEqual(status.mode, TrackingMode.STANDBY)
        self.assertFalse(status.target_detected)
        self.assertEqual(status.target_class, "")
        self.assertEqual(status.target_confidence, 0.0)
        self.assertEqual(status.pan_angle, 0.0)
        self.assertEqual(status.tilt_angle, 0.0)
        self.assertEqual(status.correction_applied, (0.0, 0.0))
        self.assertIsNone(status.last_detection_time)
        self.assertEqual(status.total_detections, 0)
        self.assertEqual(status.tracking_duration, 0.0)


class TestTrackingMode(unittest.TestCase):
    """TrackingModeEnum のテスト"""
    
    def test_tracking_mode_values(self):
        """追跡モード値テスト"""
        self.assertEqual(TrackingMode.STANDBY.value, "standby")
        self.assertEqual(TrackingMode.SCANNING.value, "scanning")
        self.assertEqual(TrackingMode.TRACKING.value, "tracking")


class TestIntegrationScenarios(unittest.TestCase):
    """統合シナリオテスト"""
    
    def setUp(self):
        """テスト前の準備"""
        logging.getLogger().setLevel(logging.WARNING)
        
        # モック環境でコーディネーター作成
        with patch('cv2.VideoCapture'), \
             patch('modules.tracking_coordinator.ServoController'), \
             patch('modules.tracking_coordinator.YOLODetector'), \
             patch('modules.tracking_coordinator.SimpleProportionalController'):
            
            self.coordinator = TrackingCoordinator(show_display=False)
    
    def tearDown(self):
        """テスト後のクリーンアップ"""
        if hasattr(self.coordinator, 'is_running') and self.coordinator.is_running:
            self.coordinator.stop_tracking()
    
    def test_complete_tracking_cycle(self):
        """完全な追跡サイクルテスト"""
        # 必要なモジュールをモック
        self.coordinator.servo_controller = Mock()
        self.coordinator.servo_controller.is_angle_safe.return_value = True  
        self.coordinator.yolo_detector = Mock()
        self.coordinator.simple_p_controller = Mock()
        self.coordinator.simple_p_controller.calculate_correction.return_value = (1.0, -0.5)
        
        # 1. 初期状態確認
        self.assertEqual(self.coordinator.status.mode, TrackingMode.STANDBY)
        
        # 2. スキャンモードに変更
        self.coordinator.status.mode = TrackingMode.SCANNING
        self.assertEqual(self.coordinator.status.mode, TrackingMode.SCANNING)
        
        # 3. 検出ありで追跡制御実行
        detections = [{
            'class_name': 'Dog',
            'confidence': 0.85,
            'bbox': (100, 100, 200, 200)
        }]
        
        self.coordinator._update_tracking_control(detections)
        
        # 4. 追跡モードに変更されることを確認
        self.assertEqual(self.coordinator.status.mode, TrackingMode.TRACKING)
        self.assertTrue(self.coordinator.status.target_detected)
        
        # 5. 検出なしで追跡制御実行
        self.coordinator._update_tracking_control([])
        
        # 6. 対象が検出されていない状態になることを確認
        self.assertFalse(self.coordinator.status.target_detected)


def run_all_tests():
    """全テストの実行"""
    print("追跡統合制御モジュール テスト実行")
    print("=" * 50)
    
    # テストスイートの作成
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # テストクラスの追加
    suite.addTests(loader.loadTestsFromTestCase(TestTrackingCoordinator))
    suite.addTests(loader.loadTestsFromTestCase(TestSystemStatus))
    suite.addTests(loader.loadTestsFromTestCase(TestTrackingMode))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegrationScenarios))
    
    # テスト実行
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 結果サマリー
    print("\n" + "=" * 50)
    print("テスト結果サマリー")
    print("=" * 50)
    print(f"実行テスト数: {result.testsRun}")
    print(f"成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"失敗: {len(result.failures)}")
    print(f"エラー: {len(result.errors)}")
    
    if result.failures:
        print("\n失敗したテスト:")
        for test, traceback in result.failures:
            print(f"- {test}: {traceback}")
    
    if result.errors:
        print("\nエラーが発生したテスト:")
        for test, traceback in result.errors:
            print(f"- {test}: {traceback}")
    
    # 成功率計算
    success_rate = (result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100
    print(f"\n成功率: {success_rate:.1f}%")
    
    return result.wasSuccessful()


if __name__ == "__main__":
    # テスト実行
    success = run_all_tests()
    
    if success:
        print("\n✓ 全てのテストが成功しました！")
        exit(0)
    else:
        print("\n✗ 一部のテストが失敗しました。")
        exit(1)