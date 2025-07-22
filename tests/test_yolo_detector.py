#!/usr/bin/env python3
"""
YOLO検出モジュール単体テスト

このテストプログラムはYOLODetectorクラスの各機能を
段階的にテストして、正常動作を確認します。

テスト項目:
1. 初期化テスト
2. モデル読み込みテスト
3. 検出機能テスト（モック）
4. 座標計算テスト
5. 追跡誤差計算テスト
6. 検出結果フィルタリングテスト
7. 統計情報取得テスト
8. 信頼度閾値変更テスト
9. エラーハンドリングテスト
10. クリーンアップテスト

使用方法:
    python tests/test_yolo_detector.py [--model MODEL_PATH]
    
    --model: 使用するYOLOモデルパス（デフォルト: yolov8n.pt）
"""

import sys
import time
import logging
import argparse
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from pathlib import Path

# プロジェクトのルートディレクトリをパスに追加
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# OpenCV、ultralyticsライブラリのモック（テスト環境用）
sys.modules['cv2'] = Mock()
sys.modules['ultralytics'] = Mock()

try:
    from modules.yolo_detector import YOLODetector, Detection, DetectorStatus, DetectorError
except ImportError as e:
    print(f"モジュールのインポートに失敗しました: {e}")
    print("プロジェクトルートディレクトリから実行してください")
    sys.exit(1)


class YOLODetectorTester:
    """YOLO検出器テストクラス"""
    
    def __init__(self, model_path: str = "yolov8n.pt"):
        """
        テスター初期化
        
        Args:
            model_path: テストに使用するモデルパス
        """
        self.model_path = model_path
        self.detector = None
        self.test_results = []
        
        # ログ設定
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        self.logger.info("モック環境でのテストを実行します")
    
    def run_all_tests(self) -> bool:
        """全テストの実行"""
        self.logger.info("=" * 60)
        self.logger.info("YOLO検出器単体テスト開始")
        self.logger.info("=" * 60)
        
        test_methods = [
            self.test_initialization,
            self.test_detection_data_class,
            self.test_model_loading,
            self.test_coordinate_calculation,
            self.test_tracking_error_calculation,
            self.test_detection_filtering,
            self.test_confidence_threshold_change,
            self.test_statistics,
            self.test_error_handling,
        ]
        
        passed_tests = 0
        total_tests = len(test_methods)
        
        for test_method in test_methods:
            try:
                if test_method():
                    passed_tests += 1
                    self.test_results.append(f"✓ {test_method.__name__}")
                else:
                    self.test_results.append(f"✗ {test_method.__name__}")
            except Exception as e:
                self.logger.error(f"テスト実行中にエラー: {test_method.__name__}: {e}")
                self.test_results.append(f"✗ {test_method.__name__} (例外: {e})")
        
        # テスト結果サマリー
        self.print_test_summary(passed_tests, total_tests)
        
        # クリーンアップテスト
        self.test_cleanup()
        
        return passed_tests == total_tests
    
    def test_initialization(self) -> bool:
        """初期化テスト"""
        self.logger.info("\n--- 初期化テスト ---")
        
        try:
            # デフォルト初期化
            self.detector = YOLODetector(model_path=self.model_path)
            
            # ステータス確認
            if self.detector.get_status() == DetectorStatus.UNINITIALIZED:
                self.logger.info("✓ 初期ステータスが正常")
            else:
                self.logger.error("✗ 初期ステータスが異常")
                return False
            
            # パラメータ確認
            if self.detector.model_path == self.model_path:
                self.logger.debug("✓ モデルパス設定正常")
            else:
                self.logger.error("✗ モデルパス設定異常")
                return False
            
            # 対象クラス確認
            expected_classes = [15, 16]  # cat, dog
            if self.detector.target_classes == expected_classes:
                self.logger.debug("✓ 対象クラス設定正常")
            else:
                self.logger.error("✗ 対象クラス設定異常")
                return False
            
            self.logger.info("✓ 初期化テスト完了")
            return True
                
        except Exception as e:
            self.logger.error(f"✗ 初期化テスト中にエラー: {e}")
            return False
    
    def test_detection_data_class(self) -> bool:
        """Detection データクラステスト"""
        self.logger.info("\n--- Detection データクラステスト ---")
        
        try:
            # テスト用検出データ
            bbox = (100, 150, 200, 250)
            detection = Detection(
                class_id=16,
                class_name="Dog",
                confidence=0.85,
                bbox=bbox
            )
            
            # 中心座標の自動計算確認
            expected_center = (150.0, 200.0)  # ((100+200)/2, (150+250)/2)
            if detection.center == expected_center:
                self.logger.debug("✓ 中心座標自動計算正常")
            else:
                self.logger.error(f"✗ 中心座標計算異常: {detection.center} != {expected_center}")
                return False
            
            # データ整合性確認
            if (detection.class_id == 16 and 
                detection.class_name == "Dog" and 
                detection.confidence == 0.85 and
                detection.bbox == bbox):
                self.logger.debug("✓ Detection データ整合性正常")
            else:
                self.logger.error("✗ Detection データ整合性異常")
                return False
            
            self.logger.info("✓ Detection データクラステスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ Detection データクラステスト中にエラー: {e}")
            return False
    
    def test_model_loading(self) -> bool:
        """モデル読み込みテスト"""
        self.logger.info("\n--- モデル読み込みテスト ---")
        
        try:
            # モックYOLOモデル
            mock_yolo = Mock()
            
            with patch('modules.yolo_detector.YOLO', return_value=mock_yolo):
                success = self.detector.load_model()
            
            if success:
                self.logger.info("✓ モデル読み込み成功（モック）")
                
                # ステータス確認
                if self.detector.get_status() == DetectorStatus.READY:
                    self.logger.debug("✓ モデル読み込み後ステータス正常")
                    return True
                else:
                    self.logger.error("✗ モデル読み込み後ステータス異常")
                    return False
            else:
                self.logger.error("✗ モデル読み込み失敗")
                return False
                
        except Exception as e:
            self.logger.error(f"✗ モデル読み込みテスト中にエラー: {e}")
            return False
    
    def test_coordinate_calculation(self) -> bool:
        """座標計算テスト"""
        self.logger.info("\n--- 座標計算テスト ---")
        
        try:
            # テストケース
            test_cases = [
                ((0, 0, 100, 100), (50.0, 50.0)),      # 正方形
                ((10, 20, 90, 80), (50.0, 50.0)),      # 長方形
                ((100, 150, 200, 250), (150.0, 200.0)) # 別サイズ
            ]
            
            for bbox, expected_center in test_cases:
                calculated_center = self.detector.calculate_center(bbox)
                
                if calculated_center == expected_center:
                    self.logger.debug(f"✓ 座標計算正常: {bbox} -> {calculated_center}")
                else:
                    self.logger.error(f"✗ 座標計算異常: {bbox} -> {calculated_center} != {expected_center}")
                    return False
            
            self.logger.info("✓ 座標計算テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 座標計算テスト中にエラー: {e}")
            return False
    
    def test_tracking_error_calculation(self) -> bool:
        """追跡誤差計算テスト"""
        self.logger.info("\n--- 追跡誤差計算テスト ---")
        
        try:
            # テスト用検出データ
            detection = Detection(
                class_id=16,
                class_name="Dog",
                confidence=0.85,
                bbox=(100, 100, 200, 200)  # 中心: (150, 150)
            )
            
            # 画像中心
            image_center = (320, 240)
            
            # 追跡誤差計算
            pan_error, tilt_error = self.detector.calculate_tracking_error(detection, image_center)
            
            # 期待値: (150 - 320, 150 - 240) = (-170, -90)
            expected_pan_error = -170.0
            expected_tilt_error = -90.0
            
            if (abs(pan_error - expected_pan_error) < 0.1 and 
                abs(tilt_error - expected_tilt_error) < 0.1):
                self.logger.debug(f"✓ 追跡誤差計算正常: Pan={pan_error}, Tilt={tilt_error}")
            else:
                self.logger.error(f"✗ 追跡誤差計算異常: Pan={pan_error}!={expected_pan_error}, Tilt={tilt_error}!={expected_tilt_error}")
                return False
            
            self.logger.info("✓ 追跡誤差計算テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 追跡誤差計算テスト中にエラー: {e}")
            return False
    
    def test_detection_filtering(self) -> bool:
        """検出結果フィルタリングテスト"""
        self.logger.info("\n--- 検出結果フィルタリングテスト ---")
        
        try:
            # テスト用検出データ
            detections = [
                Detection(16, "Dog", 0.9, (100, 100, 200, 200)),
                Detection(15, "Cat", 0.7, (300, 150, 400, 250)),
                Detection(16, "Dog", 0.6, (500, 200, 600, 300))
            ]
            
            # 最高信頼度検出結果の取得
            best_detection = self.detector.get_best_detection(detections)
            
            if best_detection and best_detection.confidence == 0.9:
                self.logger.debug("✓ 最高信頼度検出結果取得正常")
            else:
                self.logger.error("✗ 最高信頼度検出結果取得異常")
                return False
            
            # 空リストのテスト
            empty_best = self.detector.get_best_detection([])
            if empty_best is None:
                self.logger.debug("✓ 空リスト処理正常")
            else:
                self.logger.error("✗ 空リスト処理異常")
                return False
            
            self.logger.info("✓ 検出結果フィルタリングテスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 検出結果フィルタリングテスト中にエラー: {e}")
            return False
    
    def test_confidence_threshold_change(self) -> bool:
        """信頼度閾値変更テスト"""
        self.logger.info("\n--- 信頼度閾値変更テスト ---")
        
        try:
            # 有効な閾値テスト
            valid_thresholds = [0.0, 0.3, 0.5, 0.8, 1.0]
            
            for threshold in valid_thresholds:
                original_threshold = self.detector.confidence_threshold
                self.detector.set_confidence_threshold(threshold)
                
                if self.detector.confidence_threshold == threshold:
                    self.logger.debug(f"✓ 信頼度閾値変更成功: {threshold}")
                else:
                    self.logger.error(f"✗ 信頼度閾値変更失敗: {threshold}")
                    return False
            
            # 無効な閾値テスト
            invalid_thresholds = [-0.1, 1.1, 2.0]
            
            for threshold in invalid_thresholds:
                original_threshold = self.detector.confidence_threshold
                self.detector.set_confidence_threshold(threshold)
                
                # 閾値が変更されないことを確認
                if self.detector.confidence_threshold == original_threshold:
                    self.logger.debug(f"✓ 無効閾値拒否正常: {threshold}")
                else:
                    self.logger.error(f"✗ 無効閾値が受け入れられた: {threshold}")
                    return False
            
            self.logger.info("✓ 信頼度閾値変更テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 信頼度閾値変更テスト中にエラー: {e}")
            return False
    
    def test_statistics(self) -> bool:
        """統計情報テスト"""
        self.logger.info("\n--- 統計情報テスト ---")
        
        try:
            # 統計情報の取得
            stats = self.detector.get_detection_statistics()
            
            # 必須キーの確認
            required_keys = [
                'total_detections', 'recent_detection_rate', 
                'average_processing_time', 'current_fps',
                'model_path', 'confidence_threshold', 'status'
            ]
            
            for key in required_keys:
                if key not in stats:
                    self.logger.error(f"✗ 統計情報にキーが不足: {key}")
                    return False
            
            self.logger.debug("✓ 統計情報キー完整性正常")
            
            # データ型の確認
            if (isinstance(stats['total_detections'], int) and
                isinstance(stats['recent_detection_rate'], (int, float)) and
                isinstance(stats['average_processing_time'], (int, float)) and
                isinstance(stats['current_fps'], (int, float)) and
                isinstance(stats['model_path'], str) and
                isinstance(stats['confidence_threshold'], (int, float)) and
                isinstance(stats['status'], str)):
                self.logger.debug("✓ 統計情報データ型正常")
            else:
                self.logger.error("✗ 統計情報データ型異常")
                return False
            
            self.logger.info("✓ 統計情報テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 統計情報テスト中にエラー: {e}")
            return False
    
    def test_error_handling(self) -> bool:
        """エラーハンドリングテスト"""
        self.logger.info("\n--- エラーハンドリングテスト ---")
        
        try:
            # 未初期化状態でのアクセステスト
            uninit_detector = YOLODetector()
            
            # 未初期化状態では検出が空リストを返すべき
            test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            detections = uninit_detector.detect_pets(test_frame)
            
            if len(detections) == 0:
                self.logger.debug("✓ 未初期化状態での検出が適切に処理された")
            else:
                self.logger.error("✗ 未初期化状態での検出処理が異常")
                return False
            
            # ステータス確認
            if uninit_detector.get_status() == DetectorStatus.UNINITIALIZED:
                self.logger.debug("✓ 未初期化状態ステータス正常")
            else:
                self.logger.error("✗ 未初期化状態ステータス異常")
                return False
            
            self.logger.info("✓ エラーハンドリングテスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ エラーハンドリングテスト中にエラー: {e}")
            return False
    
    def test_cleanup(self) -> bool:
        """クリーンアップテスト"""
        self.logger.info("\n--- クリーンアップテスト ---")
        
        try:
            if self.detector:
                self.detector.cleanup()
                self.logger.info("✓ クリーンアップ正常完了")
                return True
        except Exception as e:
            self.logger.error(f"✗ クリーンアップでエラー: {e}")
            return False
        
        return True
    
    def print_test_summary(self, passed: int, total: int) -> None:
        """テスト結果サマリーの表示"""
        self.logger.info("\n" + "=" * 60)
        self.logger.info("テスト結果サマリー")
        self.logger.info("=" * 60)
        
        for result in self.test_results:
            print(result)
        
        success_rate = (passed / total) * 100
        self.logger.info(f"\n合格: {passed}/{total} テスト ({success_rate:.1f}%)")
        
        if passed == total:
            self.logger.info("🎉 全テストが成功しました！")
        else:
            self.logger.warning(f"⚠️  {total - passed}個のテストが失敗しました")


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='YOLO検出器単体テスト')
    parser.add_argument('--model', type=str, default='yolov8n.pt',
                       help='使用するYOLOモデルパス')
    
    args = parser.parse_args()
    
    # テスト実行
    tester = YOLODetectorTester(model_path=args.model)
    
    try:
        success = tester.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nテストが中断されました")
        if tester.detector:
            tester.detector.cleanup()
        sys.exit(1)


if __name__ == "__main__":
    main()