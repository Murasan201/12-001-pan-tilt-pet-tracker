#!/usr/bin/env python3
"""
PID制御モジュール - 追跡制御用PIDコントローラー

このモジュールはパン・チルト追跡システム用のPID制御器を実装します。
検出された対象を画面中央に維持するための制御出力を計算します。

PID制御の基本原理:
- P項（比例項）: 現在の誤差に比例した制御出力（即応性）
- I項（積分項）: 過去の誤差の蓄積を補正（定常偏差の除去）
- D項（微分項）: 誤差の変化率に対する制御（オーバーシュート抑制）

主な機能:
- 独立したパン・チルト制御器
- パラメータの動的調整
- 出力制限と安全性管理
- 制御性能の監視
- デバッグ用の内部状態取得

チューニング指針:
1. kI=0, kD=0でkPのみ調整（振動開始値の50%程度）
2. 定常偏差が残る場合kIを追加（kP/10程度から）
3. オーバーシュートが問題の場合kDを追加（kP/100程度から）

安全注意事項:
- 適切なパラメータ設定を行わないと振動や発散の可能性があります
- 出力制限を適切に設定してください
- 実機での調整は慎重に行ってください
"""

import time
import logging
import numpy as np
from typing import Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class PIDError(Exception):
    """PID制御器固有の例外"""
    pass


class PIDStatus(Enum):
    """PID制御器状態の定義"""
    UNINITIALIZED = "uninitialized"
    READY = "ready"
    RUNNING = "running"
    SATURATED = "saturated"  # 出力制限に達している状態
    ERROR = "error"


@dataclass
class PIDState:
    """PID制御器の内部状態"""
    proportional: float = 0.0      # P項の値
    integral: float = 0.0          # I項の累積値
    derivative: float = 0.0        # D項の値
    prev_error: float = 0.0        # 前回の誤差
    prev_time: float = 0.0         # 前回の時刻
    output: float = 0.0            # 制御出力
    is_saturated: bool = False     # 出力飽和フラグ


class PIDController:
    """
    PID制御器クラス - 追跡制御用
    
    パン・チルト追跡システムにおいて、検出対象の位置誤差から
    サーボ角度の補正値を計算するPID制御器です。
    """
    
    def __init__(self, 
                 kP: float = 1.0, 
                 kI: float = 0.0, 
                 kD: float = 0.0,
                 output_limits: Tuple[float, float] = (-90.0, 90.0),
                 integral_limits: Tuple[float, float] = (-50.0, 50.0),
                 sample_time: float = 0.01,
                 name: str = "PID"):
        """
        PIDコントローラー初期化
        
        Args:
            kP: 比例ゲイン（応答の速さ）
            kI: 積分ゲイン（定常偏差の除去）
            kD: 微分ゲイン（オーバーシュートの抑制）
            output_limits: 出力制限（度）
            integral_limits: 積分項制限（ワインドアップ防止）
            sample_time: サンプリング時間（秒）
            name: コントローラー名（識別用）
        """
        # PIDパラメータ
        self.kP = kP
        self.kI = kI
        self.kD = kD
        
        # 制限値
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.sample_time = sample_time
        
        # 識別情報
        self.name = name
        
        # 内部状態
        self.state = PIDState()
        self.status = PIDStatus.UNINITIALIZED
        
        # 性能監視
        self.update_count = 0
        self.saturation_count = 0
        self.performance_history = []
        
        # 統計情報
        self.start_time = time.time()
        self.total_updates = 0
        self.average_update_time = 0.0
        
        # ログ設定
        self.logger = logging.getLogger(__name__)
        
        # 初期化完了
        self.status = PIDStatus.READY
        self.state.prev_time = time.time()
        
        self.logger.debug(f"PID制御器'{self.name}'を初期化: kP={kP}, kI={kI}, kD={kD}")
    
    def update(self, error: float) -> float:
        """
        PID制御更新
        
        Args:
            error: 制御誤差（目標値 - 現在値）
            
        Returns:
            float: 制御出力（角度補正値）
        """
        if self.status == PIDStatus.ERROR:
            self.logger.warning(f"PID制御器'{self.name}'がエラー状態です")
            return 0.0
        
        update_start_time = time.time()
        
        try:
            self.status = PIDStatus.RUNNING
            current_time = time.time()
            delta_time = current_time - self.state.prev_time
            
            # サンプリング時間チェック（初回は必ず実行）
            if delta_time < self.sample_time and self.total_updates > 0:
                # サンプリング時間に達していない場合は前回の出力を返す
                return self.state.output
            
            # P項の計算（現在の誤差に比例）
            self.state.proportional = self.kP * error
            
            # I項の計算（誤差の積分、定常偏差除去）
            if delta_time > 0:
                self.state.integral += error * delta_time
                
                # 積分ワインドアップ防止（積分項制限）
                self.state.integral = np.clip(
                    self.state.integral, 
                    self.integral_limits[0], 
                    self.integral_limits[1]
                )
            
            integral_term = self.kI * self.state.integral
            
            # D項の計算（誤差の微分、オーバーシュート抑制）
            if delta_time > 0 and self.total_updates > 0:
                error_derivative = (error - self.state.prev_error) / delta_time
                self.state.derivative = self.kD * error_derivative
            else:
                self.state.derivative = 0.0
            
            # 制御出力の計算
            raw_output = self.state.proportional + integral_term + self.state.derivative
            
            # 出力制限の適用
            self.state.output = np.clip(raw_output, self.output_limits[0], self.output_limits[1])
            
            # 飽和状態の確認
            self.state.is_saturated = (abs(raw_output) > max(abs(self.output_limits[0]), abs(self.output_limits[1])))
            if self.state.is_saturated:
                self.saturation_count += 1
                self.status = PIDStatus.SATURATED
            else:
                self.status = PIDStatus.READY
            
            # 状態更新
            self.state.prev_error = error
            self.state.prev_time = current_time
            self.total_updates += 1
            
            # 性能監視
            update_time = time.time() - update_start_time
            self.average_update_time = ((self.average_update_time * (self.total_updates - 1)) + update_time) / self.total_updates
            
            # 性能履歴の記録（最新100回分）
            self.performance_history.append({
                'timestamp': current_time,
                'error': error,
                'output': self.state.output,
                'p_term': self.state.proportional,
                'i_term': integral_term,
                'd_term': self.state.derivative,
                'is_saturated': self.state.is_saturated
            })
            
            if len(self.performance_history) > 100:
                self.performance_history.pop(0)
            
            self.logger.debug(f"PID'{self.name}': Error={error:.2f}, Output={self.state.output:.2f}, "
                            f"P={self.state.proportional:.2f}, I={integral_term:.2f}, D={self.state.derivative:.2f}")
            
            return self.state.output
            
        except Exception as e:
            self.status = PIDStatus.ERROR
            self.logger.error(f"PID制御更新中にエラー: {e}")
            return 0.0
    
    def reset(self) -> None:
        """PID内部状態のリセット"""
        self.logger.info(f"PID制御器'{self.name}'をリセット")
        
        # 内部状態のクリア
        self.state.proportional = 0.0
        self.state.integral = 0.0
        self.state.derivative = 0.0
        self.state.prev_error = 0.0
        self.state.output = 0.0
        self.state.is_saturated = False
        self.state.prev_time = time.time()
        
        # 統計情報のリセット
        self.saturation_count = 0
        self.performance_history.clear()
        
        # ステータスを準備完了に変更
        if self.status != PIDStatus.ERROR:
            self.status = PIDStatus.READY
    
    def set_parameters(self, kP: float, kI: float, kD: float) -> None:
        """
        PIDパラメータの動的変更
        
        Args:
            kP: 新しい比例ゲイン
            kI: 新しい積分ゲイン
            kD: 新しい微分ゲイン
        """
        old_params = (self.kP, self.kI, self.kD)
        
        self.kP = kP
        self.kI = kI
        self.kD = kD
        
        self.logger.info(f"PID'{self.name}'パラメータ変更: "
                        f"({old_params[0]:.3f}, {old_params[1]:.3f}, {old_params[2]:.3f}) -> "
                        f"({kP:.3f}, {kI:.3f}, {kD:.3f})")
    
    def set_output_limits(self, min_output: float, max_output: float) -> None:
        """
        出力制限の設定
        
        Args:
            min_output: 最小出力値
            max_output: 最大出力値
        """
        if min_output >= max_output:
            raise PIDError(f"出力制限が無効です: min={min_output} >= max={max_output}")
        
        old_limits = self.output_limits
        self.output_limits = (min_output, max_output)
        
        self.logger.info(f"PID'{self.name}'出力制限変更: {old_limits} -> {self.output_limits}")
    
    def set_integral_limits(self, min_integral: float, max_integral: float) -> None:
        """
        積分項制限の設定（ワインドアップ防止）
        
        Args:
            min_integral: 最小積分値
            max_integral: 最大積分値
        """
        if min_integral >= max_integral:
            raise PIDError(f"積分制限が無効です: min={min_integral} >= max={max_integral}")
        
        old_limits = self.integral_limits
        self.integral_limits = (min_integral, max_integral)
        
        self.logger.info(f"PID'{self.name}'積分制限変更: {old_limits} -> {self.integral_limits}")
    
    def get_components(self) -> Dict:
        """
        P、I、D各成分の取得（デバッグ用）
        
        Returns:
            Dict: PID各成分の値
        """
        integral_term = self.kI * self.state.integral
        
        return {
            'proportional': self.state.proportional,
            'integral': integral_term,
            'derivative': self.state.derivative,
            'output': self.state.output,
            'raw_output': self.state.proportional + integral_term + self.state.derivative,
            'is_saturated': self.state.is_saturated
        }
    
    def get_parameters(self) -> Dict:
        """PIDパラメータの取得"""
        return {
            'kP': self.kP,
            'kI': self.kI,
            'kD': self.kD,
            'output_limits': self.output_limits,
            'integral_limits': self.integral_limits,
            'sample_time': self.sample_time
        }
    
    def is_stable(self, tolerance: float = 1.0, window_size: int = 10) -> bool:
        """
        制御安定性の判定
        
        Args:
            tolerance: 安定判定の許容誤差
            window_size: 判定に使用する履歴数
            
        Returns:
            bool: 安定している場合True
        """
        if len(self.performance_history) < window_size:
            return False
        
        recent_outputs = [p['output'] for p in self.performance_history[-window_size:]]
        output_variance = np.var(recent_outputs)
        
        return output_variance < tolerance
    
    def get_performance_statistics(self) -> Dict:
        """
        制御性能統計の取得
        
        Returns:
            Dict: 性能統計情報
        """
        if not self.performance_history:
            return {}
        
        recent_errors = [p['error'] for p in self.performance_history[-50:]]
        recent_outputs = [p['output'] for p in self.performance_history[-50:]]
        saturation_rate = self.saturation_count / max(self.total_updates, 1)
        
        stats = {
            'name': self.name,
            'parameters': self.get_parameters(),
            'total_updates': self.total_updates,
            'average_update_time': self.average_update_time * 1000,  # ms
            'saturation_rate': saturation_rate,
            'current_status': self.status.value,
            'recent_performance': {
                'mean_error': np.mean(np.abs(recent_errors)) if recent_errors else 0,
                'mean_output': np.mean(recent_outputs) if recent_outputs else 0,
                'output_variance': np.var(recent_outputs) if recent_outputs else 0,
                'is_stable': self.is_stable()
            }
        }
        
        return stats
    
    def get_status(self) -> PIDStatus:
        """ステータス取得"""
        return self.status
    
    def cleanup(self) -> None:
        """リソース解放"""
        try:
            self.logger.info(f"PID制御器'{self.name}'をクリーンアップ中...")
            
            # 統計情報の最終出力
            if self.total_updates > 0:
                try:
                    stats = self.get_performance_statistics()
                    self.logger.info(f"PID'{self.name}'最終統計: 更新回数={stats.get('total_updates', 0)}, "
                                   f"飽和率={stats.get('saturation_rate', 0):.1%}, "
                                   f"安定性={stats.get('recent_performance', {}).get('is_stable', False)}")
                except Exception as e:
                    self.logger.warning(f"PID'{self.name}'統計情報取得エラー: {e}")
            
            # 状態のクリア
            self.performance_history.clear()
            self.status = PIDStatus.UNINITIALIZED
            
            self.logger.info(f"PID制御器'{self.name}'のクリーンアップが完了しました")
            
        except Exception as e:
            self.logger.error(f"PID制御器'{self.name}'クリーンアップ中にエラー: {e}")


class DualPIDController:
    """
    デュアルPID制御器 - パン・チルト独立制御
    
    パンとチルトに独立したPID制御器を使用し、
    2軸の追跡制御を行います。
    """
    
    def __init__(self,
                 pan_params: Dict = None,
                 tilt_params: Dict = None):
        """
        デュアルPID制御器初期化
        
        Args:
            pan_params: パンPIDのパラメータ辞書
            tilt_params: チルトPIDのパラメータ辞書
        """
        # デフォルトパラメータ
        default_pan_params = {
            'kP': 0.8, 'kI': 0.1, 'kD': 0.05,
            'output_limits': (-90.0, 90.0),
            'name': 'Pan_PID'
        }
        
        default_tilt_params = {
            'kP': 0.8, 'kI': 0.1, 'kD': 0.05,
            'output_limits': (-45.0, 45.0),
            'name': 'Tilt_PID'
        }
        
        # パラメータのマージ
        if pan_params:
            default_pan_params.update(pan_params)
        if tilt_params:
            default_tilt_params.update(tilt_params)
        
        # PID制御器の作成
        self.pan_pid = PIDController(**default_pan_params)
        self.tilt_pid = PIDController(**default_tilt_params)
        
        self.logger = logging.getLogger(__name__)
        self.logger.info("デュアルPID制御器を初期化しました")
    
    def update(self, pan_error: float, tilt_error: float) -> Tuple[float, float]:
        """
        両軸のPID制御更新
        
        Args:
            pan_error: パン軸の誤差
            tilt_error: チルト軸の誤差
            
        Returns:
            Tuple[float, float]: (pan_output, tilt_output) 制御出力
        """
        pan_output = self.pan_pid.update(pan_error)
        tilt_output = self.tilt_pid.update(tilt_error)
        
        return (pan_output, tilt_output)
    
    def reset(self) -> None:
        """両軸PIDのリセット"""
        self.pan_pid.reset()
        self.tilt_pid.reset()
        self.logger.info("デュアルPID制御器をリセットしました")
    
    def get_statistics(self) -> Dict:
        """統合統計情報の取得"""
        return {
            'pan_stats': self.pan_pid.get_performance_statistics(),
            'tilt_stats': self.tilt_pid.get_performance_statistics()
        }
    
    def cleanup(self) -> None:
        """リソース解放"""
        self.pan_pid.cleanup()
        self.tilt_pid.cleanup()
        self.logger.info("デュアルPID制御器のクリーンアップが完了しました")


# 旧形式との互換性のためのエイリアス
def create_pid_controller(kP: float = 1.0, kI: float = 0.0, kD: float = 0.0) -> PIDController:
    """
    PID制御器のファクトリ関数
    既存コードとの互換性を保つため
    """
    return PIDController(kP=kP, kI=kI, kD=kD)


def create_dual_pid_controller() -> DualPIDController:
    """
    デュアルPID制御器のファクトリ関数
    """
    return DualPIDController()


if __name__ == "__main__":
    # モジュール単体テスト用
    import logging
    
    logging.basicConfig(level=logging.INFO)
    
    print("PIDControllerモジュール単体テスト")
    print("=" * 40)
    
    # 単体PIDテスト
    print("\n--- 単体PIDテスト ---")
    pid = PIDController(kP=1.0, kI=0.1, kD=0.05, name="TestPID")
    
    # ステップ応答テスト
    test_errors = [10.0, 8.0, 5.0, 2.0, 1.0, 0.5, 0.0, 0.0]
    
    for i, error in enumerate(test_errors):
        output = pid.update(error)
        components = pid.get_components()
        print(f"Step {i+1}: Error={error:5.1f}, Output={output:6.2f}, "
              f"P={components['proportional']:6.2f}, "
              f"I={components['integral']:6.2f}, "
              f"D={components['derivative']:6.2f}")
        time.sleep(0.1)  # シミュレーション時間間隔
    
    # デュアルPIDテスト
    print("\n--- デュアルPIDテスト ---")
    dual_pid = DualPIDController()
    
    test_cases = [
        (10.0, 5.0), (8.0, 4.0), (5.0, 2.0), (2.0, 1.0), (0.0, 0.0)
    ]
    
    for i, (pan_error, tilt_error) in enumerate(test_cases):
        pan_out, tilt_out = dual_pid.update(pan_error, tilt_error)
        print(f"Step {i+1}: Pan_Error={pan_error:5.1f} -> Output={pan_out:6.2f}, "
              f"Tilt_Error={tilt_error:5.1f} -> Output={tilt_out:6.2f}")
        time.sleep(0.1)
    
    # 統計情報表示
    print("\n--- 統計情報 ---")
    stats = dual_pid.get_statistics()
    print(f"Pan PID: 更新回数={stats['pan_stats']['total_updates']}, "
          f"安定性={stats['pan_stats']['recent_performance']['is_stable']}")
    print(f"Tilt PID: 更新回数={stats['tilt_stats']['total_updates']}, "
          f"安定性={stats['tilt_stats']['recent_performance']['is_stable']}")
    
    # クリーンアップ
    pid.cleanup()
    dual_pid.cleanup()
    print("\nテスト終了")