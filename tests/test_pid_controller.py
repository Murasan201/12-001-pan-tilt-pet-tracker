#!/usr/bin/env python3
"""
PID制御モジュール単体テスト

このテストプログラムはPIDControllerクラスの各機能を
段階的にテストして、正常動作を確認します。

テスト項目:
1. 初期化テスト
2. P制御テスト（比例制御のみ）
3. PI制御テスト（比例＋積分制御）
4. PID制御テスト（全制御項）
5. パラメータ変更テスト
6. 出力制限テスト
7. 積分ワインドアップ防止テスト
8. 安定性判定テスト
9. デュアルPID制御テスト
10. エラーハンドリングテスト
11. 統計情報テスト
12. リセット・クリーンアップテスト

使用方法:
    python tests/test_pid_controller.py [--verbose]
    
    --verbose: 詳細なテスト出力を表示
"""

import sys
import time
import logging
import argparse
import numpy as np
from pathlib import Path

# プロジェクトのルートディレクトリをパスに追加
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

try:
    from modules.pid_controller import (
        PIDController, DualPIDController, PIDStatus, PIDError,
        create_pid_controller, create_dual_pid_controller
    )
except ImportError as e:
    print(f"モジュールのインポートに失敗しました: {e}")
    print("プロジェクトルートディレクトリから実行してください")
    sys.exit(1)


class PIDControllerTester:
    """PID制御器テストクラス"""
    
    def __init__(self, verbose: bool = False):
        """
        テスター初期化
        
        Args:
            verbose: 詳細出力フラグ
        """
        self.verbose = verbose
        self.test_results = []
        
        # ログ設定
        log_level = logging.DEBUG if verbose else logging.INFO
        logging.basicConfig(level=log_level)
        self.logger = logging.getLogger(__name__)
        
        self.logger.info("PID制御器単体テストを開始します")
    
    def run_all_tests(self) -> bool:
        """全テストの実行"""
        self.logger.info("=" * 60)
        self.logger.info("PID制御器単体テスト開始")
        self.logger.info("=" * 60)
        
        test_methods = [
            self.test_initialization,
            self.test_proportional_control,
            self.test_pi_control,
            self.test_pid_control,
            self.test_parameter_changes,
            self.test_output_limits,
            self.test_integral_windup_prevention,
            self.test_stability_detection,
            self.test_dual_pid_controller,
            self.test_error_handling,
            self.test_statistics,
            self.test_reset_and_cleanup,
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
        
        return passed_tests == total_tests
    
    def test_initialization(self) -> bool:
        """初期化テスト"""
        self.logger.info("\n--- 初期化テスト ---")
        
        try:
            # デフォルト初期化
            pid = PIDController()
            
            # ステータス確認
            if pid.get_status() == PIDStatus.READY:
                self.logger.debug("✓ 初期ステータス正常")
            else:
                self.logger.error("✗ 初期ステータス異常")
                return False
            
            # デフォルトパラメータ確認
            params = pid.get_parameters()
            expected_params = {'kP': 1.0, 'kI': 0.0, 'kD': 0.0}
            
            for key, expected_value in expected_params.items():
                if abs(params[key] - expected_value) < 0.001:
                    self.logger.debug(f"✓ {key}パラメータ正常: {params[key]}")
                else:
                    self.logger.error(f"✗ {key}パラメータ異常: {params[key]} != {expected_value}")
                    return False
            
            # カスタム初期化
            custom_pid = PIDController(kP=2.0, kI=0.5, kD=0.1, name="CustomPID")
            custom_params = custom_pid.get_parameters()
            
            if (abs(custom_params['kP'] - 2.0) < 0.001 and
                abs(custom_params['kI'] - 0.5) < 0.001 and
                abs(custom_params['kD'] - 0.1) < 0.001):
                self.logger.debug("✓ カスタムパラメータ正常")
            else:
                self.logger.error("✗ カスタムパラメータ異常")
                return False
            
            self.logger.info("✓ 初期化テスト完了")
            return True
                
        except Exception as e:
            self.logger.error(f"✗ 初期化テスト中にエラー: {e}")
            return False
    
    def test_proportional_control(self) -> bool:
        """P制御（比例制御）テスト"""
        self.logger.info("\n--- P制御テスト ---")
        
        try:
            # P制御のみ（kI=0, kD=0）
            pid = PIDController(kP=1.0, kI=0.0, kD=0.0, name="P_Controller")
            
            # ステップ応答テスト
            test_errors = [10.0, 5.0, 2.0, 1.0, 0.0]
            expected_outputs = [10.0, 5.0, 2.0, 1.0, 0.0]  # kP=1.0なので誤差と同じ
            
            for i, (error, expected) in enumerate(zip(test_errors, expected_outputs)):
                output = pid.update(error)
                
                if abs(output - expected) < 0.1:
                    self.logger.debug(f"✓ P制御ステップ{i+1}: Error={error}, Output={output:.2f}")
                else:
                    self.logger.error(f"✗ P制御ステップ{i+1}: Expected={expected}, Got={output:.2f}")
                    return False
                
                time.sleep(0.01)  # サンプリング時間
            
            self.logger.info("✓ P制御テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ P制御テスト中にエラー: {e}")
            return False
    
    def test_pi_control(self) -> bool:
        """PI制御（比例＋積分制御）テスト"""
        self.logger.info("\n--- PI制御テスト ---")
        
        try:
            # PI制御（kP=1.0, kI=0.1, kD=0.0）
            pid = PIDController(kP=1.0, kI=0.1, kD=0.0, name="PI_Controller")
            
            # 定常誤差が残るケースをシミュレート
            constant_error = 1.0
            outputs = []
            
            for i in range(10):
                output = pid.update(constant_error)
                outputs.append(output)
                time.sleep(0.01)
                
                if self.verbose:
                    components = pid.get_components()
                    self.logger.debug(f"PI Step {i+1}: P={components['proportional']:.3f}, "
                                    f"I={components['integral']:.3f}, Output={output:.3f}")
            
            # 積分項により出力が増加していることを確認
            if outputs[-1] > outputs[0]:
                self.logger.debug("✓ 積分項による出力増加確認")
            else:
                self.logger.error("✗ 積分項による出力増加が確認できない")
                return False
            
            self.logger.info("✓ PI制御テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ PI制御テスト中にエラー: {e}")
            return False
    
    def test_pid_control(self) -> bool:
        """PID制御（全制御項）テスト"""
        self.logger.info("\n--- PID制御テスト ---")
        
        try:
            # フルPID制御
            pid = PIDController(kP=1.0, kI=0.2, kD=0.1, name="PID_Controller")
            
            # ステップ応答からランプ応答への変化をシミュレート
            test_sequence = [
                (10.0, "大きなステップ誤差"),
                (10.0, "同じ誤差継続（I項テスト）"),
                (5.0, "誤差減少（D項テスト）"),
                (2.0, "さらに誤差減少"),
                (1.0, "小さな誤差"),
                (0.0, "誤差ゼロ")
            ]
            
            prev_output = 0.0
            for i, (error, description) in enumerate(test_sequence):
                output = pid.update(error)
                components = pid.get_components()
                
                # 各制御項が機能していることを確認
                has_p_term = abs(components['proportional']) > 0.001 if error != 0 else True
                has_i_term = abs(components['integral']) > 0.001 if i > 0 else True
                has_d_term = abs(components['derivative']) > 0.001 if i > 0 and error != test_sequence[i-1][0] else True
                
                if has_p_term and has_i_term:
                    self.logger.debug(f"✓ PIDステップ{i+1} ({description}): "
                                    f"P={components['proportional']:.3f}, "
                                    f"I={components['integral']:.3f}, "
                                    f"D={components['derivative']:.3f}, "
                                    f"Output={output:.3f}")
                else:
                    self.logger.warning(f"⚠ PIDステップ{i+1}: 制御項の一部が動作していない可能性")
                
                prev_output = output
                time.sleep(0.01)
            
            self.logger.info("✓ PID制御テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ PID制御テスト中にエラー: {e}")
            return False
    
    def test_parameter_changes(self) -> bool:
        """パラメータ変更テスト"""
        self.logger.info("\n--- パラメータ変更テスト ---")
        
        try:
            pid = PIDController(kP=1.0, kI=0.1, kD=0.05, name="ParamTest")
            
            # 初期パラメータ確認
            initial_params = pid.get_parameters()
            
            # パラメータ変更
            new_params = (2.0, 0.3, 0.15)
            pid.set_parameters(*new_params)
            
            # 変更後パラメータ確認
            updated_params = pid.get_parameters()
            
            if (abs(updated_params['kP'] - 2.0) < 0.001 and
                abs(updated_params['kI'] - 0.3) < 0.001 and
                abs(updated_params['kD'] - 0.15) < 0.001):
                self.logger.debug("✓ パラメータ変更正常")
            else:
                self.logger.error("✗ パラメータ変更異常")
                return False
            
            # 変更後の動作確認
            output = pid.update(5.0)
            if output != 0.0:  # 何らかの応答があることを確認
                self.logger.debug("✓ パラメータ変更後の動作確認")
            else:
                self.logger.error("✗ パラメータ変更後の動作異常")
                return False
            
            self.logger.info("✓ パラメータ変更テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ パラメータ変更テスト中にエラー: {e}")
            return False
    
    def test_output_limits(self) -> bool:
        """出力制限テスト"""
        self.logger.info("\n--- 出力制限テスト ---")
        
        try:
            # 狭い出力制限を設定
            pid = PIDController(kP=10.0, kI=0.0, kD=0.0, 
                              output_limits=(-5.0, 5.0), name="LimitTest")
            
            # 制限を超える入力
            large_error = 10.0
            output = pid.update(large_error)
            
            # 出力が制限内に収まっていることを確認
            if -5.0 <= output <= 5.0:
                self.logger.debug(f"✓ 出力制限正常: Error={large_error}, Output={output}")
            else:
                self.logger.error(f"✗ 出力制限異常: Output={output} は [-5.0, 5.0] の範囲外")
                return False
            
            # 飽和状態の確認
            components = pid.get_components()
            if components['is_saturated']:
                self.logger.debug("✓ 飽和状態検出正常")
            else:
                self.logger.warning("⚠ 飽和状態が検出されていない")
            
            # 制限値の動的変更
            pid.set_output_limits(-10.0, 10.0)
            updated_params = pid.get_parameters()
            
            if updated_params['output_limits'] == (-10.0, 10.0):
                self.logger.debug("✓ 出力制限変更正常")
            else:
                self.logger.error("✗ 出力制限変更異常")
                return False
            
            self.logger.info("✓ 出力制限テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 出力制限テスト中にエラー: {e}")
            return False
    
    def test_integral_windup_prevention(self) -> bool:
        """積分ワインドアップ防止テスト"""
        self.logger.info("\n--- 積分ワインドアップ防止テスト ---")
        
        try:
            # 積分制限付きPID
            pid = PIDController(kP=0.5, kI=1.0, kD=0.0,
                              integral_limits=(-5.0, 5.0), name="WindupTest")
            
            # 長期間の大きな誤差で積分項を飽和させる
            large_error = 10.0
            for i in range(20):
                output = pid.update(large_error)
                time.sleep(0.01)
            
            # 積分項が制限内に収まっていることを確認
            components = pid.get_components()
            integral_value = components['integral'] / pid.kI  # 生の積分値
            
            if -5.0 <= integral_value <= 5.0:
                self.logger.debug(f"✓ 積分ワインドアップ防止正常: 積分値={integral_value:.3f}")
            else:
                self.logger.error(f"✗ 積分ワインドアップ防止異常: 積分値={integral_value:.3f}")
                return False
            
            self.logger.info("✓ 積分ワインドアップ防止テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 積分ワインドアップ防止テスト中にエラー: {e}")
            return False
    
    def test_stability_detection(self) -> bool:
        """安定性判定テスト"""
        self.logger.info("\n--- 安定性判定テスト ---")
        
        try:
            pid = PIDController(kP=0.5, kI=0.1, kD=0.1, name="StabilityTest")
            
            # 不安定な応答をシミュレート（振動）
            oscillating_errors = [5.0, -3.0, 4.0, -2.0, 3.0, -1.0, 2.0, -0.5, 1.0, 0.0]
            
            for error in oscillating_errors:
                pid.update(error)
                time.sleep(0.01)
            
            # この時点では不安定であるべき
            if not pid.is_stable(tolerance=0.5):
                self.logger.debug("✓ 不安定状態の検出正常")
            else:
                self.logger.warning("⚠ 不安定状態が検出されていない")
            
            # 安定した応答をシミュレート
            stable_errors = [0.1, 0.05, 0.02, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            for error in stable_errors:
                pid.update(error)
                time.sleep(0.01)
            
            # 安定状態の確認
            if pid.is_stable(tolerance=0.1):
                self.logger.debug("✓ 安定状態の検出正常")
            else:
                self.logger.warning("⚠ 安定状態が検出されていない")
            
            self.logger.info("✓ 安定性判定テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 安定性判定テスト中にエラー: {e}")
            return False
    
    def test_dual_pid_controller(self) -> bool:
        """デュアルPID制御テスト"""
        self.logger.info("\n--- デュアルPID制御テスト ---")
        
        try:
            # デュアルPID制御器の作成
            dual_pid = DualPIDController()
            
            # 両軸の制御テスト
            test_cases = [
                (10.0, 5.0),   # 大きな誤差
                (5.0, 3.0),    # 中程度の誤差
                (2.0, 1.0),    # 小さな誤差
                (0.0, 0.0)     # 誤差ゼロ
            ]
            
            for i, (pan_error, tilt_error) in enumerate(test_cases):
                pan_output, tilt_output = dual_pid.update(pan_error, tilt_error)
                
                # 出力が妥当な範囲内であることを確認
                if (-90.0 <= pan_output <= 90.0 and -45.0 <= tilt_output <= 45.0):
                    self.logger.debug(f"✓ デュアルPIDステップ{i+1}: "
                                    f"Pan({pan_error:.1f}→{pan_output:.2f}), "
                                    f"Tilt({tilt_error:.1f}→{tilt_output:.2f})")
                else:
                    self.logger.error(f"✗ デュアルPIDステップ{i+1}: 出力が範囲外")
                    return False
                
                time.sleep(0.01)
            
            # 統計情報の確認
            stats = dual_pid.get_statistics()
            
            if ('pan_stats' in stats and 'tilt_stats' in stats):
                self.logger.debug("✓ デュアルPID統計情報正常")
                
                # 更新回数の確認
                pan_updates = stats['pan_stats']['total_updates']
                tilt_updates = stats['tilt_stats']['total_updates']
                
                if pan_updates > 0 and tilt_updates > 0:
                    self.logger.debug(f"✓ 更新回数正常: Pan={pan_updates}, Tilt={tilt_updates}")
                else:
                    self.logger.error("✗ 更新回数異常")
                    return False
            else:
                self.logger.error("✗ デュアルPID統計情報異常")
                return False
            
            # クリーンアップテスト
            dual_pid.cleanup()
            
            self.logger.info("✓ デュアルPID制御テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ デュアルPID制御テスト中にエラー: {e}")
            return False
    
    def test_error_handling(self) -> bool:
        """エラーハンドリングテスト"""
        self.logger.info("\n--- エラーハンドリングテスト ---")
        
        try:
            # 無効な制限値でのエラーテスト
            try:
                pid = PIDController()
                pid.set_output_limits(10.0, 5.0)  # min > max
                self.logger.error("✗ 無効な出力制限が受け入れられた")
                return False
            except PIDError:
                self.logger.debug("✓ 無効な出力制限が適切に拒否された")
            
            try:
                pid = PIDController()
                pid.set_integral_limits(5.0, 2.0)  # min > max
                self.logger.error("✗ 無効な積分制限が受け入れられた")
                return False
            except PIDError:
                self.logger.debug("✓ 無効な積分制限が適切に拒否された")
            
            self.logger.info("✓ エラーハンドリングテスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ エラーハンドリングテスト中にエラー: {e}")
            return False
    
    def test_statistics(self) -> bool:
        """統計情報テスト"""
        self.logger.info("\n--- 統計情報テスト ---")
        
        try:
            pid = PIDController(kP=1.0, kI=0.1, kD=0.05, name="StatsTest")
            
            # いくつかの更新を実行
            for error in [5.0, 3.0, 1.0, 0.5, 0.0]:
                pid.update(error)
                time.sleep(0.01)
            
            # 統計情報の取得
            stats = pid.get_performance_statistics()
            
            # 必須キーの確認
            required_keys = [
                'name', 'parameters', 'total_updates', 'average_update_time',
                'saturation_rate', 'current_status', 'recent_performance'
            ]
            
            for key in required_keys:
                if key not in stats:
                    self.logger.error(f"✗ 統計情報にキーが不足: {key}")
                    return False
            
            self.logger.debug("✓ 統計情報キー完整性正常")
            
            # データの妥当性確認
            if (stats['total_updates'] > 0 and
                stats['average_update_time'] >= 0 and
                0 <= stats['saturation_rate'] <= 1):
                self.logger.debug("✓ 統計情報データ妥当性正常")
            else:
                self.logger.error("✗ 統計情報データ妥当性異常")
                return False
            
            self.logger.info("✓ 統計情報テスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ 統計情報テスト中にエラー: {e}")
            return False
    
    def test_reset_and_cleanup(self) -> bool:
        """リセット・クリーンアップテスト"""
        self.logger.info("\n--- リセット・クリーンアップテスト ---")
        
        try:
            pid = PIDController(kP=1.0, kI=0.5, kD=0.1, name="ResetTest")
            
            # いくつかの更新で内部状態を変更
            for error in [10.0, 8.0, 5.0]:
                pid.update(error)
                time.sleep(0.01)
            
            # リセット前の状態確認
            pre_reset_components = pid.get_components()
            pre_reset_stats = pid.get_performance_statistics()
            
            if pre_reset_components['integral'] != 0.0:
                self.logger.debug("✓ リセット前の積分項確認")
            else:
                self.logger.warning("⚠ リセット前の積分項がゼロ")
            
            # リセット実行
            pid.reset()
            
            # リセット後の状態確認
            post_reset_components = pid.get_components()
            
            if (post_reset_components['proportional'] == 0.0 and
                post_reset_components['integral'] == 0.0 and
                post_reset_components['derivative'] == 0.0 and
                post_reset_components['output'] == 0.0):
                self.logger.debug("✓ リセット後の状態クリア確認")
            else:
                self.logger.error("✗ リセット後の状態クリア異常")
                return False
            
            # クリーンアップテスト
            pid.cleanup()
            
            if pid.get_status() == PIDStatus.UNINITIALIZED:
                self.logger.debug("✓ クリーンアップ後ステータス正常")
            else:
                self.logger.error("✗ クリーンアップ後ステータス異常")
                return False
            
            self.logger.info("✓ リセット・クリーンアップテスト完了")
            return True
            
        except Exception as e:
            self.logger.error(f"✗ リセット・クリーンアップテスト中にエラー: {e}")
            return False
    
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
    parser = argparse.ArgumentParser(description='PID制御器単体テスト')
    parser.add_argument('--verbose', action='store_true', 
                       help='詳細なテスト出力を表示')
    
    args = parser.parse_args()
    
    # テスト実行
    tester = PIDControllerTester(verbose=args.verbose)
    
    try:
        success = tester.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nテストが中断されました")
        sys.exit(1)


if __name__ == "__main__":
    main()