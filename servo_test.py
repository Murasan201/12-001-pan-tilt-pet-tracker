#!/usr/bin/env python3
"""
サーボテストプログラム - パン・チルト機構の動作確認

このプログラムはAdafruit PCA9685 PWMサーボドライバとSG90サーボモータを使用して
パン・チルト機構の動作テストを実行します。

動作内容:
- パン（左右）方向に3回往復
- チルト（上下）方向に3回往復
- 各動作間に適切なウェイト時間
- テスト完了後は中央位置に復帰

安全注意事項:
- プログラム実行前にハードウェア接続を確認してください
- 動作中はサーボの可動範囲に障害物がないことを確認してください
- 異常音や振動を感じた場合は即座に停止してください
"""

import time
import sys
import signal
from typing import Optional

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo
except ImportError as e:
    print(f"必要なライブラリがインストールされていません: {e}")
    print("以下のコマンドでインストールしてください:")
    print("pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor")
    sys.exit(1)


class ServoController:
    """サーボ制御クラス"""
    
    def __init__(self):
        """サーボドライバの初期化"""
        self.i2c = None
        self.pca = None
        self.pan_servo = None
        self.tilt_servo = None
        
        # サーボ角度の設定
        self.PAN_CENTER = 90      # パン中央位置
        self.TILT_CENTER = 90     # チルト中央位置
        self.PAN_MIN = 0          # パン最小角度
        self.PAN_MAX = 180        # パン最大角度
        self.TILT_MIN = 45        # チルト最小角度（安全範囲）
        self.TILT_MAX = 135       # チルト最大角度（安全範囲）
        
        # 動作パラメータ
        self.MOVE_DELAY = 1.0     # 動作間隔（秒）
        self.SETTLE_DELAY = 0.5   # 動作後の安定待機時間（秒）
    
    def initialize(self) -> bool:
        """
        サーボドライバとサーボモータの初期化
        
        Returns:
            bool: 初期化成功時True、失敗時False
        """
        try:
            print("サーボドライバを初期化中...")
            
            # I2Cバスの初期化
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # PCA9685の初期化
            self.pca = PCA9685(self.i2c)
            self.pca.frequency = 50  # 50Hz（サーボ標準周波数）
            
            # サーボモータの初期化
            # チャンネル0: パンサーボ、チャンネル1: チルトサーボ
            self.pan_servo = servo.Servo(self.pca.channels[0])
            self.tilt_servo = servo.Servo(self.pca.channels[1])
            
            # 中央位置に移動
            print("サーボを中央位置に設定中...")
            self.pan_servo.angle = self.PAN_CENTER
            self.tilt_servo.angle = self.TILT_CENTER
            time.sleep(2)  # 初期位置への移動待機
            
            print("サーボドライバの初期化が完了しました")
            return True
            
        except Exception as e:
            print(f"サーボドライバの初期化に失敗しました: {e}")
            return False
    
    def move_pan(self, angle: float) -> None:
        """
        パンサーボを指定角度に移動
        
        Args:
            angle: 移動先角度（0-180度）
        """
        # 角度の範囲チェック
        angle = max(self.PAN_MIN, min(self.PAN_MAX, angle))
        
        print(f"パン: {angle}度に移動")
        self.pan_servo.angle = angle
        time.sleep(self.SETTLE_DELAY)
    
    def move_tilt(self, angle: float) -> None:
        """
        チルトサーボを指定角度に移動
        
        Args:
            angle: 移動先角度（45-135度の安全範囲）
        """
        # 角度の範囲チェック（安全範囲内）
        angle = max(self.TILT_MIN, min(self.TILT_MAX, angle))
        
        print(f"チルト: {angle}度に移動")
        self.tilt_servo.angle = angle
        time.sleep(self.SETTLE_DELAY)
    
    def test_pan_movement(self, cycles: int = 3) -> None:
        """
        パン（左右）動作のテスト
        
        Args:
            cycles: テスト回数
        """
        print(f"\n=== パン動作テスト開始（{cycles}回往復） ===")
        
        for i in range(cycles):
            print(f"パンテスト {i+1}/{cycles}回目:")
            
            # 右に移動
            self.move_pan(self.PAN_MAX)
            time.sleep(self.MOVE_DELAY)
            
            # 中央に戻る
            self.move_pan(self.PAN_CENTER)
            time.sleep(self.MOVE_DELAY)
            
            # 左に移動
            self.move_pan(self.PAN_MIN)
            time.sleep(self.MOVE_DELAY)
            
            # 中央に戻る
            self.move_pan(self.PAN_CENTER)
            time.sleep(self.MOVE_DELAY)
        
        print("パン動作テスト完了")
    
    def test_tilt_movement(self, cycles: int = 3) -> None:
        """
        チルト（上下）動作のテスト
        
        Args:
            cycles: テスト回数
        """
        print(f"\n=== チルト動作テスト開始（{cycles}回往復） ===")
        
        for i in range(cycles):
            print(f"チルトテスト {i+1}/{cycles}回目:")
            
            # 上に移動
            self.move_tilt(self.TILT_MAX)
            time.sleep(self.MOVE_DELAY)
            
            # 中央に戻る
            self.move_tilt(self.TILT_CENTER)
            time.sleep(self.MOVE_DELAY)
            
            # 下に移動
            self.move_tilt(self.TILT_MIN)
            time.sleep(self.MOVE_DELAY)
            
            # 中央に戻る
            self.move_tilt(self.TILT_CENTER)
            time.sleep(self.MOVE_DELAY)
        
        print("チルト動作テスト完了")
    
    def return_to_center(self) -> None:
        """サーボを中央位置に復帰"""
        print("\n中央位置に復帰中...")
        self.pan_servo.angle = self.PAN_CENTER
        self.tilt_servo.angle = self.TILT_CENTER
        time.sleep(1)
        print("中央位置に復帰しました")
    
    def cleanup(self) -> None:
        """リソースのクリーンアップ"""
        try:
            if self.pca:
                print("サーボドライバをシャットダウン中...")
                # PWM出力を停止
                for channel in self.pca.channels:
                    channel.duty_cycle = 0
                self.pca.deinit()
                print("サーボドライバのクリーンアップが完了しました")
        except Exception as e:
            print(f"クリーンアップ中にエラーが発生しました: {e}")


def signal_handler(signum, frame):
    """シグナルハンドラ（Ctrl+C対応）"""
    print("\n\n割り込みが検出されました。安全停止中...")
    if 'controller' in globals():
        controller.return_to_center()
        controller.cleanup()
    sys.exit(0)


def main():
    """メイン関数"""
    print("=" * 60)
    print("サーボテストプログラム - パン・チルト機構動作確認")
    print("=" * 60)
    print()
    print("注意事項:")
    print("- ハードウェア接続を確認してください")
    print("- サーボの可動範囲に障害物がないことを確認してください")
    print("- 異常を感じた場合はCtrl+Cで停止してください")
    print()
    
    # シグナルハンドラの設定
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # ユーザー確認
    try:
        response = input("準備が完了したらEnterキーを押してください（中止する場合はCtrl+C）: ")
    except KeyboardInterrupt:
        print("\nテストが中止されました")
        return
    
    global controller
    controller = ServoController()
    
    try:
        # サーボドライバの初期化
        if not controller.initialize():
            print("初期化に失敗しました。ハードウェア接続を確認してください。")
            return
        
        print("\nテスト開始...")
        time.sleep(1)
        
        # パン動作テスト
        controller.test_pan_movement(cycles=3)
        
        # テスト間の待機
        time.sleep(2)
        
        # チルト動作テスト
        controller.test_tilt_movement(cycles=3)
        
        # 中央位置に復帰
        controller.return_to_center()
        
        print("\n" + "=" * 60)
        print("すべてのテストが正常に完了しました！")
        print("=" * 60)
        
    except Exception as e:
        print(f"\nテスト中にエラーが発生しました: {e}")
        
    finally:
        # リソースのクリーンアップ
        controller.cleanup()


if __name__ == "__main__":
    main()