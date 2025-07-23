#!/usr/bin/env python3
"""
Simple P制御モジュール - パン・チルト追跡用比例制御器

このモジュールは一般的なSimple P制御を実装し、
パン・チルト追跡システムで安定した制御を提供します。

PID制御に比べてSimple P制御の利点:
- 実装の簡潔性: 約10行のコードで実装可能
- 制御周期問題の回避: Python/Raspberry Pi環境での不安定性を回避
- 実証済み実績: 一般的なパン・チルト追跡システムでの成功実績
- デバッグ容易性: 動作が直感的で問題特定が簡単
- 計算負荷軽減: Raspberry Piのリソースを効率的に活用

適用場面:
- ペット見守りシステム（低頻度動作）
- 教育・学習用途
- プロトタイプ・実証段階
- 静止画撮影が主目的のアプリケーション

技術背景:
Simple P制御は比例項のみを使用する制御方式で、
多くのパン・チルト追跡システムで成功している実証済みアプローチです。
"""

import time
import logging
import numpy as np
from typing import Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class SimplePError(Exception):
    """Simple P制御器固有の例外"""
    pass


class SimplePStatus(Enum):
    """Simple P制御器状態の定義"""
    UNINITIALIZED = "uninitialized"
    READY = "ready"
    RUNNING = "running"
    ERROR = "error"


@dataclass
class SimplePState:
    """Simple P制御器の内部状態"""
    last_error: Tuple[float, float] = (0.0, 0.0)      # 前回の誤差（パン、チルト）
    last_output: Tuple[float, float] = (0.0, 0.0)     # 前回の出力（パン、チルト）
    last_update_time: float = 0.0                     # 前回の更新時刻
    total_corrections: int = 0                        # 総補正回数


class SimpleProportionalController:
    """
    Simple P制御器クラス - 一般的な比例制御方式
    
    パン・チルト追跡システムにおいて、検出対象の位置誤差から
    サーボ角度の補正値を直接計算するシンプルな比例制御器です。
    
    一般的なパン・チルト追跡システムで実証済みのパラメータとアルゴリズムを使用し、
    Raspberry Pi Python環境での制御周期不安定性を回避します。
    """
    
    def __init__(self, 
                 image_width: int = 640,
                 image_height: int = 480,
                 pan_gain: float = 0.0156,      # 10.0/640 (一般的な推奨値)
                 tilt_gain: float = 0.0208,     # 10.0/480 (一般的な推奨値)
                 max_correction: float = 15.0,  # 1回の最大補正角度（度）
                 deadband: float = 5.0,         # 不感帯（pixel）
                 name: str = "SimpleP"):
        """
        Simple P制御器初期化
        
        Args:
            image_width: 画像幅（pixel）
            image_height: 画像高さ（pixel）
            pan_gain: パン制御ゲイン（一般的な推奨値）
            tilt_gain: チルト制御ゲイン（一般的な推奨値）
            max_correction: 1回の最大補正角度（度）
            deadband: 不感帯（pixel、この範囲内は補正しない）
            name: コントローラー名（識別用）
        """
        # 画像パラメータ
        self.image_width = image_width
        self.image_height = image_height
        self.image_center = (image_width // 2, image_height // 2)
        
        # 制御パラメータ（一般的な実証値）
        self.pan_gain = pan_gain      # Kp_pan
        self.tilt_gain = tilt_gain    # Kp_tilt
        self.max_correction = max_correction
        self.deadband = deadband
        
        # 識別情報
        self.name = name
        
        # 内部状態
        self.state = SimplePState()
        self.status = SimplePStatus.UNINITIALIZED
        
        # 性能監視
        self.performance_history = []
        self.start_time = time.time()
        
        # ログ設定
        self.logger = logging.getLogger(__name__)
        
        # 初期化完了
        self.status = SimplePStatus.READY
        self.state.last_update_time = time.time()
        
        self.logger.info(f"Simple P制御器'{self.name}'を初期化: "
                        f"pan_gain={pan_gain:.4f}, tilt_gain={tilt_gain:.4f}, "
                        f"max_correction={max_correction}度")
    
    def calculate_correction(self, detection_center: Tuple[float, float]) -> Tuple[float, float]:
        """
        検出中心から角度補正値を計算（一般的な比例制御方式）
        
        Args:
            detection_center: 検出対象の中心座標 (x, y)
            
        Returns:
            Tuple[float, float]: (pan_correction, tilt_correction) 角度補正値（度）
        """
        if self.status == SimplePStatus.ERROR:
            self.logger.warning(f"Simple P制御器'{self.name}'がエラー状態です")
            return (0.0, 0.0)
        
        try:
            self.status = SimplePStatus.RUNNING
            current_time = time.time()
            
            # 誤差計算（画像座標系）
            x_error = detection_center[0] - self.image_center[0]
            y_error = detection_center[1] - self.image_center[1]
            
            # 不感帯処理（小さな誤差は無視）
            if abs(x_error) < self.deadband:
                x_error = 0.0
            if abs(y_error) < self.deadband:
                y_error = 0.0
            
            # Simple P制御（比例制御のみ、一般的な比例制御方式）
            pan_correction = x_error * self.pan_gain
            tilt_correction = -y_error * self.tilt_gain  # Y軸反転（カメラ座標系→サーボ座標系）
            
            # 補正角度制限（安全性確保）
            pan_correction = np.clip(pan_correction, -self.max_correction, self.max_correction)
            tilt_correction = np.clip(tilt_correction, -self.max_correction, self.max_correction)
            
            # 状態更新
            self.state.last_error = (x_error, y_error)
            self.state.last_output = (pan_correction, tilt_correction)
            self.state.last_update_time = current_time
            self.state.total_corrections += 1
            
            # 性能履歴の記録（最新50回分）
            self.performance_history.append({
                'timestamp': current_time,
                'detection_center': detection_center,
                'error': (x_error, y_error),
                'correction': (pan_correction, tilt_correction),
                'is_in_deadband': (abs(x_error) < self.deadband and abs(y_error) < self.deadband)
            })
            
            if len(self.performance_history) > 50:
                self.performance_history.pop(0)
            
            self.logger.debug(f"Simple P'{self.name}': "
                            f"Error=({x_error:.1f}, {y_error:.1f})pixel, "
                            f"Correction=({pan_correction:.2f}, {tilt_correction:.2f})度")
            
            self.status = SimplePStatus.READY
            return (pan_correction, tilt_correction)
            
        except Exception as e:
            self.status = SimplePStatus.ERROR
            self.logger.error(f"Simple P制御計算中にエラー: {e}")
            return (0.0, 0.0)
    
    def calculate_tracking_error(self, detection_bbox: Tuple[float, float, float, float]) -> Tuple[float, float]:
        """
        バウンディングボックスから制御誤差を計算
        
        Args:
            detection_bbox: (x1, y1, x2, y2) バウンディングボックス
            
        Returns:
            Tuple[float, float]: (pan_error, tilt_error) 制御誤差（pixel）
        """
        try:
            # バウンディングボックスの中心を計算
            bbox_center_x = (detection_bbox[0] + detection_bbox[2]) / 2
            bbox_center_y = (detection_bbox[1] + detection_bbox[3]) / 2
            
            # 画像中心からのずれを計算
            pan_error = bbox_center_x - self.image_center[0]
            tilt_error = bbox_center_y - self.image_center[1]
            
            return (pan_error, tilt_error)
            
        except Exception as e:
            self.logger.error(f"制御誤差計算エラー: {e}")
            return (0.0, 0.0)
    
    def set_gains(self, pan_gain: float, tilt_gain: float) -> None:
        """
        制御ゲインの動的変更
        
        Args:
            pan_gain: 新しいパン制御ゲイン
            tilt_gain: 新しいチルト制御ゲイン
        """
        old_gains = (self.pan_gain, self.tilt_gain)
        self.pan_gain = pan_gain
        self.tilt_gain = tilt_gain
        
        self.logger.info(f"Simple P'{self.name}'ゲイン変更: "
                        f"({old_gains[0]:.4f}, {old_gains[1]:.4f}) -> "
                        f"({pan_gain:.4f}, {tilt_gain:.4f})")
    
    def set_max_correction(self, max_correction: float) -> None:
        """
        最大補正角度の設定
        
        Args:
            max_correction: 新しい最大補正角度（度）
        """
        old_max = self.max_correction
        self.max_correction = max_correction
        
        self.logger.info(f"Simple P'{self.name}'最大補正角度変更: {old_max}度 -> {max_correction}度")
    
    def set_deadband(self, deadband: float) -> None:
        """
        不感帯の設定
        
        Args:
            deadband: 新しい不感帯（pixel）
        """
        old_deadband = self.deadband
        self.deadband = deadband
        
        self.logger.info(f"Simple P'{self.name}'不感帯変更: {old_deadband}pixel -> {deadband}pixel")
    
    def get_parameters(self) -> Dict:
        """制御パラメータの取得"""
        return {
            'pan_gain': self.pan_gain,
            'tilt_gain': self.tilt_gain,
            'max_correction': self.max_correction,
            'deadband': self.deadband,
            'image_size': (self.image_width, self.image_height),
            'image_center': self.image_center
        }
    
    def get_state(self) -> Dict:
        """内部状態の取得"""
        return {
            'last_error': self.state.last_error,
            'last_output': self.state.last_output,
            'last_update_time': self.state.last_update_time,
            'total_corrections': self.state.total_corrections,
            'status': self.status.value
        }
    
    def get_performance_statistics(self) -> Dict:
        """
        制御性能統計の取得
        
        Returns:
            Dict: 性能統計情報
        """
        if not self.performance_history:
            return {
                'name': self.name,
                'total_corrections': self.state.total_corrections,
                'status': self.status.value
            }
        
        # 最近の性能データを分析
        recent_errors_x = [p['error'][0] for p in self.performance_history[-20:]]
        recent_errors_y = [p['error'][1] for p in self.performance_history[-20:]]
        recent_corrections_pan = [p['correction'][0] for p in self.performance_history[-20:]]
        recent_corrections_tilt = [p['correction'][1] for p in self.performance_history[-20:]]
        
        deadband_count = sum(1 for p in self.performance_history[-20:] if p['is_in_deadband'])
        
        stats = {
            'name': self.name,
            'parameters': self.get_parameters(),
            'total_corrections': self.state.total_corrections,
            'status': self.status.value,
            'recent_performance': {
                'mean_error_x': np.mean(np.abs(recent_errors_x)) if recent_errors_x else 0,
                'mean_error_y': np.mean(np.abs(recent_errors_y)) if recent_errors_y else 0,
                'mean_correction_pan': np.mean(np.abs(recent_corrections_pan)) if recent_corrections_pan else 0,
                'mean_correction_tilt': np.mean(np.abs(recent_corrections_tilt)) if recent_corrections_tilt else 0,
                'deadband_rate': deadband_count / min(len(self.performance_history), 20),
                'tracking_precision': {
                    'x_variance': np.var(recent_errors_x) if recent_errors_x else 0,
                    'y_variance': np.var(recent_errors_y) if recent_errors_y else 0
                }
            }
        }
        
        return stats
    
    def reset(self) -> None:
        """制御器状態のリセット"""
        self.logger.info(f"Simple P制御器'{self.name}'をリセット")
        
        # 内部状態のクリア
        self.state.last_error = (0.0, 0.0)
        self.state.last_output = (0.0, 0.0)
        self.state.last_update_time = time.time()
        self.state.total_corrections = 0
        
        # 性能履歴のクリア
        self.performance_history.clear()
        
        # ステータスを準備完了に変更
        if self.status != SimplePStatus.ERROR:
            self.status = SimplePStatus.READY
    
    def get_status(self) -> SimplePStatus:
        """ステータス取得"""
        return self.status
    
    def cleanup(self) -> None:
        """リソース解放"""
        try:
            self.logger.info(f"Simple P制御器'{self.name}'をクリーンアップ中...")
            
            # 統計情報の最終出力
            if self.state.total_corrections > 0:
                try:
                    stats = self.get_performance_statistics()
                    recent_perf = stats.get('recent_performance', {})
                    self.logger.info(f"Simple P'{self.name}'最終統計: "
                                   f"補正回数={stats.get('total_corrections', 0)}, "
                                   f"平均誤差=({recent_perf.get('mean_error_x', 0):.1f}, "
                                   f"{recent_perf.get('mean_error_y', 0):.1f})pixel, "
                                   f"不感帯率={recent_perf.get('deadband_rate', 0):.1%}")
                except Exception as e:
                    self.logger.warning(f"Simple P'{self.name}'統計情報取得エラー: {e}")
            
            # 状態のクリア
            self.performance_history.clear()
            self.status = SimplePStatus.UNINITIALIZED
            
            self.logger.info(f"Simple P制御器'{self.name}'のクリーンアップが完了しました")
            
        except Exception as e:
            self.logger.error(f"Simple P制御器'{self.name}'クリーンアップ中にエラー: {e}")


class DualSimplePController:
    """
    デュアルSimple P制御器 - パン・チルト独立制御
    
    パンとチルトに独立したSimple P制御器を使用し、
    2軸の追跡制御を行います。
    """
    
    def __init__(self,
                 image_width: int = 640,
                 image_height: int = 480,
                 pan_gain: float = 0.0156,
                 tilt_gain: float = 0.0208):
        """
        デュアルSimple P制御器初期化
        
        Args:
            image_width: 画像幅
            image_height: 画像高さ
            pan_gain: パン制御ゲイン
            tilt_gain: チルト制御ゲイン
        """
        # 統一されたパラメータで制御器を作成
        self.controller = SimpleProportionalController(
            image_width=image_width,
            image_height=image_height,
            pan_gain=pan_gain,
            tilt_gain=tilt_gain,
            name="DualSimpleP"
        )
        
        self.logger = logging.getLogger(__name__)
        self.logger.info("デュアルSimple P制御器を初期化しました")
    
    def update(self, detection_center: Tuple[float, float]) -> Tuple[float, float]:
        """
        両軸のSimple P制御更新
        
        Args:
            detection_center: 検出中心座標 (x, y)
            
        Returns:
            Tuple[float, float]: (pan_correction, tilt_correction) 制御出力
        """
        return self.controller.calculate_correction(detection_center)
    
    def update_from_bbox(self, detection_bbox: Tuple[float, float, float, float]) -> Tuple[float, float]:
        """
        バウンディングボックスからの制御更新
        
        Args:
            detection_bbox: (x1, y1, x2, y2) バウンディングボックス
            
        Returns:
            Tuple[float, float]: (pan_correction, tilt_correction) 制御出力
        """
        # バウンディングボックスから中心座標を計算
        center_x = (detection_bbox[0] + detection_bbox[2]) / 2
        center_y = (detection_bbox[1] + detection_bbox[3]) / 2
        
        return self.controller.calculate_correction((center_x, center_y))
    
    def reset(self) -> None:
        """制御器のリセット"""
        self.controller.reset()
        self.logger.info("デュアルSimple P制御器をリセットしました")
    
    def get_statistics(self) -> Dict:
        """統計情報の取得"""
        return self.controller.get_performance_statistics()
    
    def cleanup(self) -> None:
        """リソース解放"""
        self.controller.cleanup()
        self.logger.info("デュアルSimple P制御器のクリーンアップが完了しました")


# ファクトリ関数
def create_simple_p_controller(image_width: int = 640, 
                              image_height: int = 480,
                              pan_gain: float = 0.0156,
                              tilt_gain: float = 0.0208) -> SimpleProportionalController:
    """
    Simple P制御器のファクトリ関数
    
    Args:
        image_width: 画像幅
        image_height: 画像高さ  
        pan_gain: パン制御ゲイン（PiCar-X準拠デフォルト）
        tilt_gain: チルト制御ゲイン（PiCar-X準拠デフォルト）
    
    Returns:
        SimpleProportionalController: 初期化済み制御器
    """
    return SimpleProportionalController(
        image_width=image_width,
        image_height=image_height,
        pan_gain=pan_gain,
        tilt_gain=tilt_gain
    )


def create_dual_simple_p_controller(image_width: int = 640,
                                   image_height: int = 480) -> DualSimplePController:
    """
    デュアルSimple P制御器のファクトリ関数
    
    Returns:
        DualSimplePController: 初期化済みデュアル制御器
    """
    return DualSimplePController(image_width=image_width, image_height=image_height)


if __name__ == "__main__":
    # モジュール単体テスト用
    import logging
    
    logging.basicConfig(level=logging.INFO)
    
    print("SimpleProportionalControllerモジュール単体テスト")
    print("=" * 50)
    
    # 単体Simple Pテスト
    print("\n--- Simple P制御テスト ---")
    simple_p = SimpleProportionalController(name="TestSimpleP")
    
    # テストケース: 画面上の様々な位置
    test_cases = [
        (320, 240),  # 画面中央（誤差なし）
        (400, 240),  # 右に80pixel
        (240, 240),  # 左に80pixel
        (320, 180),  # 上に60pixel
        (320, 300),  # 下に60pixel
        (480, 360),  # 右下角
        (160, 120),  # 左上角
    ]
    
    print("画面中心: (320, 240)")
    print("テスト形式: 検出位置 -> 誤差(pixel) -> 補正(度)")
    print("-" * 60)
    
    for i, (x, y) in enumerate(test_cases):
        correction = simple_p.calculate_correction((x, y))
        error = simple_p.get_state()['last_error']
        print(f"Test {i+1}: ({x:3d}, {y:3d}) -> Error=({error[0]:6.1f}, {error[1]:6.1f}) -> "
              f"Correction=({correction[0]:6.2f}, {correction[1]:6.2f})")
    
    # バウンディングボックステスト
    print("\n--- バウンディングボックステスト ---")
    test_bboxes = [
        (300, 220, 340, 260),  # 画面中央の小さなボックス
        (200, 180, 280, 240),  # 左上寄りのボックス
        (360, 280, 440, 340),  # 右下寄りのボックス
    ]
    
    for i, bbox in enumerate(test_bboxes):
        correction = simple_p.update_from_bbox(bbox)
        center_x = (bbox[0] + bbox[2]) / 2
        center_y = (bbox[1] + bbox[3]) / 2
        print(f"BBox {i+1}: {bbox} -> Center=({center_x:.1f}, {center_y:.1f}) -> "
              f"Correction=({correction[0]:6.2f}, {correction[1]:6.2f})")
    
    # デュアルSimple Pテスト
    print("\n--- デュアルSimple P制御テスト ---")
    dual_simple_p = DualSimplePController()
    
    for i, (x, y) in enumerate(test_cases[:5]):  # 最初の5ケースをテスト
        correction = dual_simple_p.update((x, y))
        print(f"Dual {i+1}: ({x:3d}, {y:3d}) -> Correction=({correction[0]:6.2f}, {correction[1]:6.2f})")
    
    # 統計情報表示
    print("\n--- 統計情報 ---")
    stats = simple_p.get_performance_statistics()
    recent_perf = stats.get('recent_performance', {})
    print(f"Simple P統計: 補正回数={stats.get('total_corrections', 0)}, "
          f"平均誤差X={recent_perf.get('mean_error_x', 0):.1f}pixel, "
          f"平均誤差Y={recent_perf.get('mean_error_y', 0):.1f}pixel")
    
    dual_stats = dual_simple_p.get_statistics()
    dual_recent = dual_stats.get('recent_performance', {})
    print(f"Dual Simple P統計: 補正回数={dual_stats.get('total_corrections', 0)}, "
          f"不感帯率={dual_recent.get('deadband_rate', 0):.1%}")
    
    # クリーンアップ
    simple_p.cleanup()
    dual_simple_p.cleanup()
    print("\nテスト終了")