#!/usr/bin/env python3
"""
サーボ制御モジュール - パン・チルトサーボの精密制御

このモジュールはAdafruit PCA9685 PWMサーボドライバを使用して
SG90サーボモータによるパン・チルト機構を制御します。

主な機能:
- パン・チルトサーボの独立制御
- 安全範囲内での動作制限
- 角度変換と座標系管理
- エラーハンドリングと安全停止
- テスト機能とキャリブレーション

安全注意事項:
- 動作範囲制限を必ず遵守してください
- 異常検出時は即座に停止してください
- 初期化失敗時はハードウェア接続を確認してください
"""

import time
import sys
import logging
from typing import Optional, Tuple
from enum import Enum

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo
except ImportError as e:
    print(f"必要なライブラリがインストールされていません: {e}")
    print("以下のコマンドでインストールしてください:")
    print("pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor")
    raise


class ServoControllerError(Exception):
    """サーボコントローラー固有の例外"""
    pass


class ServoStatus(Enum):
    """サーボ状態の定義"""
    UNINITIALIZED = "uninitialized"
    INITIALIZING = "initializing"
    READY = "ready"
    MOVING = "moving"
    ERROR = "error"


class ServoController:
    """
    サーボ制御クラス - パン・チルトサーボの精密制御
    
    このクラスは既存のservo_test.pyの機能を拡張し、
    追跡システムで使用するための高精度制御機能を提供します。
    """
    
    def __init__(self, 
                 i2c_address: int = 0x40,
                 pwm_frequency: int = 50,
                 pan_channel: int = 0,
                 tilt_channel: int = 1,
                 pan_range: Tuple[float, float] = (-90, 90),
                 tilt_range: Tuple[float, float] = (-45, 45)):
        """
        サーボコントローラー初期化
        
        Args:
            i2c_address (int): PCA9685のI2Cアドレス（16進数、デフォルト: 0x40）
            pwm_frequency (int): PWM周波数（Hz、サーボ標準: 50Hz）
            pan_channel (int): パンサーボのチャンネル番号（PCA9685の0〜15）
            tilt_channel (int): チルトサーボのチャンネル番号（PCA9685の0〜15）
            pan_range (Tuple[float, float]): パン動作範囲（度、-90°から+90°）
            tilt_range (Tuple[float, float]): チルト動作範囲（度、-45°から+45°）
        """
        # ハードウェア設定
        self.i2c_address = i2c_address
        self.pwm_frequency = pwm_frequency
        self.pan_channel = pan_channel
        self.tilt_channel = tilt_channel
        
        # 動作範囲設定（追跡システム用の角度範囲）
        self.pan_min, self.pan_max = pan_range
        self.tilt_min, self.tilt_max = tilt_range
        
        # ハードウェアインスタンス
        self.i2c = None
        self.pca = None
        self.pan_servo = None
        self.tilt_servo = None
        
        # 現在角度の記録（追跡用）
        self.current_pan_angle = 0.0
        self.current_tilt_angle = 0.0
        
        # ステータス管理
        self.status = ServoStatus.UNINITIALIZED
        
        # 動作パラメータ
        self.move_speed = 0.5  # 動作速度（秒）
        self.settle_time = 0.1  # 安定待機時間（秒）
        
        # ログ設定
        self.logger = logging.getLogger(__name__)
    
    def initialize(self) -> bool:
        """
        サーボドライバの初期化
        
        Returns:
            bool: 初期化成功時True、失敗時False
        """
        try:
            self.status = ServoStatus.INITIALIZING
            self.logger.info("サーボドライバを初期化中...")
            
            # I2Cバスの初期化
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # PCA9685の初期化
            self.pca = PCA9685(self.i2c, address=self.i2c_address)
            self.pca.frequency = self.pwm_frequency
            
            # サーボモータの初期化
            self.pan_servo = servo.Servo(self.pca.channels[self.pan_channel])
            self.tilt_servo = servo.Servo(self.pca.channels[self.tilt_channel])
            
            # 中央位置に移動（追跡システムの基準位置）
            self.logger.info("サーボを中央位置に設定中...")
            self._set_servo_angles(0.0, 0.0)  # 中央位置（0度、0度）
            time.sleep(2.0)  # 初期位置への移動待機
            
            self.current_pan_angle = 0.0
            self.current_tilt_angle = 0.0
            self.status = ServoStatus.READY
            
            self.logger.info("サーボドライバの初期化が完了しました")
            return True
            
        except Exception as e:
            self.status = ServoStatus.ERROR
            self.logger.error(f"サーボドライバの初期化に失敗しました: {e}")
            return False
    
    def set_pan_angle(self, angle: float) -> bool:
        """
        パン角度設定
        
        Args:
            angle: パン角度（度）-90°～+90°
            
        Returns:
            bool: 設定成功時True、失敗時False
        """
        if not self._is_ready():
            return False
        
        if not self._is_pan_angle_safe(angle):
            self.logger.warning(f"パン角度が安全範囲外です: {angle}度")
            return False
        
        try:
            self.status = ServoStatus.MOVING
            servo_angle = self._pan_angle_to_servo(angle)
            self.pan_servo.angle = servo_angle
            self.current_pan_angle = angle
            time.sleep(self.settle_time)
            self.status = ServoStatus.READY
            
            self.logger.debug(f"パン角度を設定: {angle}度 (サーボ角度: {servo_angle}度)")
            return True
            
        except Exception as e:
            self.status = ServoStatus.ERROR
            self.logger.error(f"パン角度設定中にエラー: {e}")
            return False
    
    def set_tilt_angle(self, angle: float) -> bool:
        """
        チルト角度設定
        
        Args:
            angle: チルト角度（度）-45°～+45°
            
        Returns:
            bool: 設定成功時True、失敗時False
        """
        if not self._is_ready():
            return False
        
        if not self._is_tilt_angle_safe(angle):
            self.logger.warning(f"チルト角度が安全範囲外です: {angle}度")
            return False
        
        try:
            self.status = ServoStatus.MOVING
            servo_angle = self._tilt_angle_to_servo(angle)
            self.tilt_servo.angle = servo_angle
            self.current_tilt_angle = angle
            time.sleep(self.settle_time)
            self.status = ServoStatus.READY
            
            self.logger.debug(f"チルト角度を設定: {angle}度 (サーボ角度: {servo_angle}度)")
            return True
            
        except Exception as e:
            self.status = ServoStatus.ERROR
            self.logger.error(f"チルト角度設定中にエラー: {e}")
            return False
    
    def set_angles(self, pan_angle: float, tilt_angle: float) -> bool:
        """
        パン・チルト同時設定
        
        Args:
            pan_angle: パン角度（度）
            tilt_angle: チルト角度（度）
            
        Returns:
            bool: 設定成功時True、失敗時False
        """
        if not self._is_ready():
            return False
        
        if not self.is_angle_safe(pan_angle, tilt_angle):
            self.logger.warning(f"角度が安全範囲外です: Pan={pan_angle}度, Tilt={tilt_angle}度")
            return False
        
        try:
            self.status = ServoStatus.MOVING
            self._set_servo_angles(pan_angle, tilt_angle)
            self.current_pan_angle = pan_angle
            self.current_tilt_angle = tilt_angle
            time.sleep(self.settle_time)
            self.status = ServoStatus.READY
            
            self.logger.debug(f"角度を同時設定: Pan={pan_angle}度, Tilt={tilt_angle}度")
            return True
            
        except Exception as e:
            self.status = ServoStatus.ERROR
            self.logger.error(f"角度同時設定中にエラー: {e}")
            return False
    
    def get_current_angles(self) -> Tuple[float, float]:
        """
        現在の角度取得
        
        Returns:
            Tuple[float, float]: (pan_angle, tilt_angle) 現在のパン・チルト角度
        """
        return (self.current_pan_angle, self.current_tilt_angle)
    
    def move_to_center(self) -> bool:
        """
        中央位置への移動
        
        Returns:
            bool: 移動成功時True、失敗時False
        """
        self.logger.info("中央位置に移動中...")
        return self.set_angles(0.0, 0.0)
    
    def test_movement(self, cycles: int = 1) -> bool:
        """
        動作テスト（既存servo_test.pyの機能）
        
        Args:
            cycles: テスト回数
            
        Returns:
            bool: テスト成功時True、失敗時False
        """
        if not self._is_ready():
            self.logger.error("サーボが準備状態ではありません")
            return False
        
        try:
            self.logger.info(f"サーボ動作テストを開始 ({cycles}回)")
            
            for i in range(cycles):
                self.logger.info(f"テスト {i+1}/{cycles}回目")
                
                # パンテスト
                test_angles = [45, 0, -45, 0]  # 右、中央、左、中央
                for angle in test_angles:
                    if not self.set_pan_angle(angle):
                        return False
                    time.sleep(0.5)
                
                # チルトテスト  
                test_angles = [30, 0, -30, 0]  # 上、中央、下、中央
                for angle in test_angles:
                    if not self.set_tilt_angle(angle):
                        return False
                    time.sleep(0.5)
            
            # 中央位置に復帰
            self.move_to_center()
            self.logger.info("サーボ動作テストが完了しました")
            return True
            
        except Exception as e:
            self.logger.error(f"動作テスト中にエラー: {e}")
            return False
    
    def is_angle_safe(self, pan: float, tilt: float) -> bool:
        """
        角度の安全性チェック
        
        Args:
            pan: パン角度
            tilt: チルト角度
            
        Returns:
            bool: 安全な場合True
        """
        return self._is_pan_angle_safe(pan) and self._is_tilt_angle_safe(tilt)
    
    def emergency_stop(self) -> None:
        """緊急停止"""
        try:
            self.logger.warning("緊急停止を実行中...")
            self.status = ServoStatus.ERROR
            
            # 中央位置に緊急移動
            if self.pan_servo and self.tilt_servo:
                self._set_servo_angles(0.0, 0.0)
            
            self.logger.warning("緊急停止が完了しました")
            
        except Exception as e:
            self.logger.error(f"緊急停止中にエラー: {e}")
    
    def cleanup(self) -> None:
        """リソース解放"""
        try:
            if self.status != ServoStatus.UNINITIALIZED:
                self.logger.info("サーボドライバをクリーンアップ中...")
                
                # 中央位置に復帰
                if self.pan_servo and self.tilt_servo:
                    self._set_servo_angles(0.0, 0.0)
                    time.sleep(1.0)
                
                # PWM出力を停止
                if self.pca:
                    for channel in self.pca.channels:
                        channel.duty_cycle = 0
                    self.pca.deinit()
                
                self.status = ServoStatus.UNINITIALIZED
                self.logger.info("サーボドライバのクリーンアップが完了しました")
                
        except Exception as e:
            self.logger.error(f"クリーンアップ中にエラー: {e}")
    
    def get_status(self) -> ServoStatus:
        """ステータス取得"""
        return self.status
    
    def set_move_speed(self, speed: float) -> None:
        """動作速度設定"""
        if 0.1 <= speed <= 2.0:
            self.move_speed = speed
            self.logger.debug(f"動作速度を設定: {speed}秒")
        else:
            self.logger.warning(f"動作速度が範囲外です: {speed}")
    
    # プライベートメソッド
    
    def _is_ready(self) -> bool:
        """準備状態チェック"""
        if self.status != ServoStatus.READY:
            self.logger.warning(f"サーボが準備状態ではありません: {self.status}")
            return False
        return True
    
    def _is_pan_angle_safe(self, angle: float) -> bool:
        """パン角度安全性チェック"""
        return self.pan_min <= angle <= self.pan_max
    
    def _is_tilt_angle_safe(self, angle: float) -> bool:
        """チルト角度安全性チェック"""
        return self.tilt_min <= angle <= self.tilt_max
    
    def _pan_angle_to_servo(self, angle: float) -> float:
        """
        パン角度をサーボ角度に変換
        追跡システム座標系（-90°～+90°）からサーボ座標系（0°～180°）への変換
        """
        return angle + 90.0  # -90°～+90° → 0°～180°
    
    def _tilt_angle_to_servo(self, angle: float) -> float:
        """
        チルト角度をサーボ角度に変換
        追跡システム座標系（-45°～+45°）からサーボ座標系（45°～135°）への変換
        """
        return angle + 90.0  # -45°～+45° → 45°～135°
    
    def _set_servo_angles(self, pan_angle: float, tilt_angle: float) -> None:
        """サーボ角度の直接設定（内部使用）"""
        pan_servo_angle = self._pan_angle_to_servo(pan_angle)
        tilt_servo_angle = self._tilt_angle_to_servo(tilt_angle)
        
        self.pan_servo.angle = pan_servo_angle
        self.tilt_servo.angle = tilt_servo_angle


# 旧形式との互換性のためのエイリアス
def create_servo_controller() -> ServoController:
    """
    サーボコントローラーのファクトリ関数
    既存コードとの互換性を保つため
    """
    return ServoController()


if __name__ == "__main__":
    # モジュール単体テスト用
    import logging
    
    logging.basicConfig(level=logging.INFO)
    
    print("ServoControllerモジュール単体テスト")
    print("=" * 40)
    
    controller = ServoController()
    
    try:
        if controller.initialize():
            print("初期化成功")
            
            if controller.test_movement(cycles=1):
                print("動作テスト成功")
            else:
                print("動作テスト失敗")
        else:
            print("初期化失敗")
    
    except KeyboardInterrupt:
        print("\nテスト中断")
    
    finally:
        controller.cleanup()
        print("テスト終了")