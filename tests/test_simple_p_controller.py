#!/usr/bin/env python3
"""
Simple P制御器のテストモジュール

このモジュールはSimpleProportionalControllerクラスの
全機能をテストし、正常動作を確認します。

テスト項目:
- 基本的な比例制御の動作
- パラメータ設定と変更
- バウンディングボックス処理
- 不感帯機能
- エラーハンドリング
- 性能統計
"""

import unittest
import sys
import os
import logging
import time
from unittest.mock import patch, MagicMock

# プロジェクトルートをパスに追加
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from modules.simple_p_controller import (
    SimpleProportionalController,
    SimplePStatus,
    SimplePError,
    create_simple_p_controller
)


class TestSimpleProportionalController(unittest.TestCase):
    """SimpleProportionalControllerクラスのテスト"""
    
    def setUp(self):
        """テスト前の準備"""
        # ログレベルを警告以上に設定（テスト中のログ出力を抑制）
        logging.getLogger().setLevel(logging.WARNING)
        
        # 標準的な設定でコントローラーを作成
        self.controller = SimpleProportionalController(
            image_width=640,
            image_height=480,
            pan_gain=0.0156,
            tilt_gain=0.0208,
            name="TestController"
        )
    
    def tearDown(self):
        """テスト後のクリーンアップ"""
        self.controller.cleanup()
    
    def test_initialization(self):
        """初期化テスト"""
        # 初期化状態の確認
        self.assertEqual(self.controller.status, SimplePStatus.READY)
        self.assertEqual(self.controller.image_width, 640)
        self.assertEqual(self.controller.image_height, 480)
        self.assertEqual(self.controller.image_center, (320, 240))
        self.assertEqual(self.controller.pan_gain, 0.0156)
        self.assertEqual(self.controller.tilt_gain, 0.0208)
        self.assertEqual(self.controller.name, "TestController")
    
    def test_basic_proportional_control(self):
        """基本的な比例制御テスト"""
        # 画面中央（誤差なし）
        correction = self.controller.calculate_correction((320, 240))
        self.assertEqual(correction, (0.0, 0.0))
        
        # 右に100pixel移動（正の誤差）
        correction = self.controller.calculate_correction((420, 240))
        expected_pan = 100 * 0.0156  # 1.56度
        self.assertAlmostEqual(correction[0], expected_pan, places=3)
        self.assertAlmostEqual(correction[1], 0.0, places=3)
        
        # 左に100pixel移動（負の誤差）
        correction = self.controller.calculate_correction((220, 240))
        expected_pan = -100 * 0.0156  # -1.56度
        self.assertAlmostEqual(correction[0], expected_pan, places=3)
        self.assertAlmostEqual(correction[1], 0.0, places=3)
        
        # 上に80pixel移動（Y軸反転考慮）
        correction = self.controller.calculate_correction((320, 160))
        expected_tilt = -(-80) * 0.0208  # +1.664度（Y軸反転）
        self.assertAlmostEqual(correction[0], 0.0, places=3)
        self.assertAlmostEqual(correction[1], expected_tilt, places=3)
        
        # 下に80pixel移動（Y軸反転考慮）
        correction = self.controller.calculate_correction((320, 320))
        expected_tilt = -(80) * 0.0208  # -1.664度（Y軸反転）
        self.assertAlmostEqual(correction[0], 0.0, places=3)
        self.assertAlmostEqual(correction[1], expected_tilt, places=3)
    
    def test_diagonal_movement(self):
        """対角線移動のテスト"""
        # 右下に移動
        correction = self.controller.calculate_correction((420, 320))
        expected_pan = 100 * 0.0156   # 1.56度
        expected_tilt = -80 * 0.0208  # -1.664度
        
        self.assertAlmostEqual(correction[0], expected_pan, places=3)
        self.assertAlmostEqual(correction[1], expected_tilt, places=3)
    
    def test_max_correction_limit(self):
        """最大補正角度制限テスト"""
        # 極端に大きな誤差（画面外）
        correction = self.controller.calculate_correction((1000, 1000))
        
        # 最大補正角度（15度）を超えないことを確認
        self.assertLessEqual(abs(correction[0]), 15.0)
        self.assertLessEqual(abs(correction[1]), 15.0)
    
    def test_deadband_functionality(self):
        """不感帯機能テスト"""
        # デフォルト不感帯は5pixel
        # 不感帯内の小さな誤差
        correction = self.controller.calculate_correction((323, 243))  # 3pixelの誤差
        self.assertEqual(correction, (0.0, 0.0))
        
        # 不感帯外の誤差
        correction = self.controller.calculate_correction((327, 247))  # 7pixelの誤差
        self.assertNotEqual(correction, (0.0, 0.0))
    
    def test_bbox_tracking_error_calculation(self):
        """バウンディングボックス制御誤差計算テスト"""
        # 中央のバウンディングボックス
        bbox = (300, 220, 340, 260)  # 中心(320, 240)
        error = self.controller.calculate_tracking_error(bbox)
        self.assertEqual(error, (0.0, 0.0))
        
        # 右上のバウンディングボックス
        bbox = (350, 180, 390, 220)  # 中心(370, 200)
        error = self.controller.calculate_tracking_error(bbox)
        expected_x = 370 - 320  # 50pixel
        expected_y = 200 - 240  # -40pixel
        self.assertEqual(error, (expected_x, expected_y))
    
    def test_parameter_changes(self):
        """パラメータ変更テスト"""
        # ゲイン変更
        self.controller.set_gains(0.02, 0.025)
        self.assertEqual(self.controller.pan_gain, 0.02)
        self.assertEqual(self.controller.tilt_gain, 0.025)
        
        # 最大補正角度変更
        self.controller.set_max_correction(20.0)
        self.assertEqual(self.controller.max_correction, 20.0)
        
        # 不感帯変更
        self.controller.set_deadband(10.0)
        self.assertEqual(self.controller.deadband, 10.0)
    
    def test_state_tracking(self):
        """状態追跡テスト"""
        # 初期状態確認
        state = self.controller.get_state()
        self.assertEqual(state['last_error'], (0.0, 0.0))
        self.assertEqual(state['total_corrections'], 0)
        
        # 補正実行後の状態確認
        self.controller.calculate_correction((350, 280))
        state = self.controller.get_state()
        self.assertEqual(state['last_error'], (30.0, 40.0))
        self.assertEqual(state['total_corrections'], 1)
    
    def test_performance_statistics(self):
        """性能統計テスト"""
        # 複数回の補正実行
        test_positions = [(350, 280), (300, 200), (400, 300), (280, 260)]
        for pos in test_positions:
            self.controller.calculate_correction(pos)
        
        # 統計情報取得
        stats = self.controller.get_performance_statistics()
        
        # 基本統計の確認
        self.assertEqual(stats['name'], 'TestController')
        self.assertEqual(stats['total_corrections'], 4)
        self.assertIn('recent_performance', stats)
        self.assertIn('parameters', stats)
        
        # 最近の性能データの確認
        recent = stats['recent_performance']
        self.assertGreater(recent['mean_error_x'], 0)
        self.assertGreater(recent['mean_error_y'], 0)
    
    def test_reset_functionality(self):
        """リセット機能テスト"""
        # 補正実行
        self.controller.calculate_correction((400, 300))
        
        # リセット前の状態確認
        self.assertNotEqual(self.controller.state.total_corrections, 0)
        
        # リセット実行
        self.controller.reset()
        
        # リセット後の状態確認
        self.assertEqual(self.controller.state.total_corrections, 0)
        self.assertEqual(self.controller.state.last_error, (0.0, 0.0))
        self.assertEqual(len(self.controller.performance_history), 0)
    
    def test_error_handling(self):
        """エラーハンドリングテスト"""
        # 不正な検出中心座標でのエラーハンドリング
        with patch.object(self.controller, 'logger') as mock_logger:
            # NumPyエラーを発生させる
            with patch('numpy.clip', side_effect=Exception("Test error")):
                correction = self.controller.calculate_correction((100, 100))
                
                # エラー時は(0.0, 0.0)を返すことを確認
                self.assertEqual(correction, (0.0, 0.0))
                self.assertEqual(self.controller.status, SimplePStatus.ERROR)



class TestFactoryFunctions(unittest.TestCase):
    """ファクトリ関数のテスト"""
    
    def test_create_simple_p_controller(self):
        """Simple P制御器ファクトリ関数テスト"""
        controller = create_simple_p_controller(
            image_width=800,
            image_height=600,
            pan_gain=0.02,
            tilt_gain=0.025
        )
        
        self.assertEqual(controller.image_width, 800)
        self.assertEqual(controller.image_height, 600)
        self.assertEqual(controller.pan_gain, 0.02)
        self.assertEqual(controller.tilt_gain, 0.025)
        
        controller.cleanup()
    


class TestIntegrationScenarios(unittest.TestCase):
    """統合シナリオテスト"""
    
    def setUp(self):
        """テスト前の準備"""
        logging.getLogger().setLevel(logging.WARNING)
        self.controller = SimpleProportionalController(name="IntegrationTest")
    
    def tearDown(self):
        """テスト後のクリーンアップ"""
        self.controller.cleanup()
    
    def test_tracking_scenario(self):
        """追跡シナリオテスト（シンプルな可動域確認）"""
        # 基本的な位置での補正値確認
        test_positions = [
            (200, 180),  # 左上
            (400, 300),  # 右下  
            (320, 240),  # 中央（補正なし）
        ]
        
        corrections = []
        for pos in test_positions:
            correction = self.controller.calculate_correction(pos)
            corrections.append(correction)
        
        # 中央では補正が0になることを確認
        self.assertEqual(corrections[2], (0.0, 0.0))
        
        # 左上と右下では補正が発生することを確認
        self.assertNotEqual(corrections[0], (0.0, 0.0))
        self.assertNotEqual(corrections[1], (0.0, 0.0))
    
    def test_extreme_conditions(self):
        """極端な条件テスト（可動域制限確認）"""
        # 画面外の極端な位置
        extreme_positions = [
            (-100, -100),  # 画面外左上
            (1000, 1000),  # 画面外右下
            (320, -200),   # 画面外上
            (320, 800),    # 画面外下
        ]
        
        for pos in extreme_positions:
            correction = self.controller.calculate_correction(pos)
            
            # 最大補正角度を超えないことを確認（可動域制限）
            self.assertLessEqual(abs(correction[0]), 15.0)
            self.assertLessEqual(abs(correction[1]), 15.0)
    
    def test_performance_consistency(self):
        """計算一貫性テスト（シンプルな繰り返し確認）"""
        # 同じ位置で複数回実行して結果が一貫していることを確認
        test_position = (400, 300)
        results = []
        
        for _ in range(5):
            correction = self.controller.calculate_correction(test_position)
            results.append(correction)
        
        # 全ての結果が同じであることを確認（シンプルな計算なので一貫性があるはず）
        for result in results[1:]:
            self.assertEqual(result, results[0])


def run_all_tests():
    """全テストの実行"""
    print("Simple P制御器モジュール テスト実行")
    print("=" * 50)
    
    # テストスイートの作成
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # テストクラスの追加
    suite.addTests(loader.loadTestsFromTestCase(TestSimpleProportionalController))
    suite.addTests(loader.loadTestsFromTestCase(TestFactoryFunctions))
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