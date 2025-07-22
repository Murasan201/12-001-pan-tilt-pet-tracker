#!/usr/bin/env python3
"""
サーボ初期位置調整プログラム - メカニカル組付け前の位置出し

このプログラムは、パン・チルトサーボモータの可動域中央位置に正確に位置決めするためのツールです。
サーボとメカニカル部品の組付け前に実行し、正確な中央位置でサーボホーンを取り付けてください。

主な機能:
- パンサーボ（左右）の中央位置（90度）設定
- チルトサーボ（上下）の中央位置（90度）設定
- 個別サーボの位置確認とテスト
- 組付け作業のための安定した位置保持
- 安全な電源OFF機能

使用方法:
1. サーボをPCA9685ドライバに接続
2. このプログラムを実行
3. 中央位置を確認後、サーボホーンを取り付け
4. メカニカル部品を組み付け

注意事項:
- 組付け前の位置出し専用プログラムです
- サーボホーン取り付け時は正確な中央位置を維持してください
- 組付け作業中はサーボに無理な力を加えないでください
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


class ServoPositionSetup:
    """サーボ初期位置調整クラス"""
    
    def __init__(self):
        """初期化"""
        self.i2c = None
        self.pca = None
        self.pan_servo = None
        self.tilt_servo = None
        
        # サーボ角度の設定（servo_test.pyと同じ設定）
        self.PAN_CENTER = 90      # パン中央位置
        self.TILT_CENTER = 90     # チルト中央位置
        self.PAN_MIN = 0          # パン最小角度
        self.PAN_MAX = 180        # パン最大角度
        self.TILT_MIN = 45        # チルト最小角度（安全範囲）
        self.TILT_MAX = 135       # チルト最大角度（安全範囲）
        
        # 動作パラメータ（servo_test.pyと同じ設定）
        self.MOVE_DELAY = 1.0     # 動作間隔（秒）
        self.SETTLE_DELAY = 0.5   # 動作後の安定待機時間（秒）
        
        # 終了フラグ
        self.is_running = True
        
    def setup_signal_handlers(self):
        """シグナルハンドラーの設定"""
        def signal_handler(signum, frame):
            print("\n割り込み信号を受信しました。安全に終了します...")
            self.cleanup()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
    def initialize_hardware(self) -> bool:
        """ハードウェアの初期化（servo_test.pyと同じ方法）"""
        try:
            print("サーボドライバを初期化中...")
            
            # I2Cバスの初期化
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # PCA9685の初期化（servo_test.pyと同じ）
            self.pca = PCA9685(self.i2c)
            self.pca.frequency = 50  # 50Hz（サーボ標準周波数）
            
            # サーボモータの初期化（servo_test.pyと同じ）
            # チャンネル0: パンサーボ、チャンネル1: チルトサーボ
            self.pan_servo = servo.Servo(self.pca.channels[0])
            self.tilt_servo = servo.Servo(self.pca.channels[1])
            
            # 中央位置に移動（servo_test.pyと同じ処理）
            print("サーボを中央位置に設定中...")
            self.pan_servo.angle = self.PAN_CENTER
            self.tilt_servo.angle = self.TILT_CENTER
            time.sleep(2)  # 初期位置への移動待機
            
            print("サーボドライバの初期化が完了しました")
            return True
            
        except Exception as e:
            print(f"サーボドライバの初期化に失敗しました: {e}")
            return False
    
    def move_to_center_position(self):
        """両サーボを中央位置に移動（servo_test.pyの方法使用）"""
        try:
            print(f"\n両サーボを中央位置（パン:{self.PAN_CENTER}度、チルト:{self.TILT_CENTER}度）に移動中...")
            
            # servo_test.pyと同じ方法で移動
            print(f"パン: {self.PAN_CENTER}度に移動")
            self.pan_servo.angle = self.PAN_CENTER
            time.sleep(self.SETTLE_DELAY)
            
            print(f"チルト: {self.TILT_CENTER}度に移動")
            self.tilt_servo.angle = self.TILT_CENTER
            time.sleep(self.SETTLE_DELAY)
            
            print("✓ 中央位置への移動が完了しました")
            
        except Exception as e:
            print(f"中央位置移動エラー: {e}")
            raise
    
    def test_individual_servo(self, servo_name: str, servo_obj, center_angle: int) -> bool:
        """個別サーボのテスト（servo_test.pyの方法を参考）"""
        try:
            print(f"\n=== {servo_name}サーボのテスト ===")
            
            # 中央位置確認
            print(f"{servo_name}: {center_angle}度に移動")
            servo_obj.angle = center_angle
            time.sleep(self.SETTLE_DELAY)
            print(f"✓ {servo_name}: 中央位置（{center_angle}度）")
            
            # 軽微な動作テスト（±10度）
            test_angles = [center_angle + 10, center_angle, center_angle - 10, center_angle]
            
            for angle in test_angles:
                print(f"{servo_name}: {angle}度に移動")
                servo_obj.angle = angle
                time.sleep(self.SETTLE_DELAY)
                print(f"✓ {servo_name}: {angle}度に移動完了")
            
            print(f"✓ {servo_name}サーボのテスト完了（中央位置で停止）")
            return True
            
        except Exception as e:
            print(f"{servo_name}サーボテストエラー: {e}")
            return False
    
    def interactive_menu(self):
        """対話型メニュー"""
        while self.is_running:
            print("\n" + "="*60)
            print("サーボ初期位置調整プログラム - メニュー")
            print("="*60)
            print("1. 両サーボを中央位置に移動")
            print("2. パンサーボの動作テスト")
            print("3. チルトサーボの動作テスト")
            print("4. 位置保持モード（組付け作業用）")
            print("5. プログラム終了")
            print("-"*60)
            
            try:
                choice = input("選択してください (1-5): ").strip()
                
                if choice == "1":
                    self.move_to_center_position()
                    
                elif choice == "2":
                    self.test_individual_servo("パン", self.pan_servo, self.PAN_CENTER)
                    
                elif choice == "3":
                    self.test_individual_servo("チルト", self.tilt_servo, self.TILT_CENTER)
                    
                elif choice == "4":
                    self.position_hold_mode()
                    
                elif choice == "5":
                    print("プログラムを終了します...")
                    self.is_running = False
                    
                else:
                    print("無効な選択です。1-5の数字を入力してください。")
                    
            except KeyboardInterrupt:
                print("\n操作が中止されました")
                self.is_running = False
            except EOFError:
                print("\n入力が終了されました")
                self.is_running = False
    
    def position_hold_mode(self):
        """位置保持モード（組付け作業用）"""
        print("\n" + "="*50)
        print("位置保持モード - メカニカル組付け作業用")
        print("="*50)
        print("サーボを中央位置で安定保持します")
        print("組付け作業中はCtrl+Cで終了してください")
        print("-"*50)
        
        try:
            # 中央位置に移動して保持
            self.move_to_center_position()
            
            print("✓ 位置保持中... （Ctrl+Cで終了）")
            print("\n組付け手順:")
            print("1. サーボホーンが中央位置（90度）にあることを確認")
            print("2. メカニカル部品をサーボホーンに正確に取り付け")
            print("3. ネジで固定")
            print("4. 動作に支障がないことを確認")
            print("5. Ctrl+Cでプログラム終了")
            
            # 無限ループで位置保持
            while True:
                time.sleep(1.0)
                
        except KeyboardInterrupt:
            print("\n位置保持モードを終了しました")
    
    def cleanup(self):
        """リソースクリーンアップ"""
        try:
            if self.pca:
                print("サーボドライバをシャットダウン中...")
                self.pca.deinit()
                print("✓ クリーンアップが完了しました")
        except Exception as e:
            print(f"クリーンアップエラー: {e}")


def print_header():
    """ヘッダー表示"""
    print("="*70)
    print("サーボ初期位置調整プログラム - メカニカル組付け前位置出し")
    print("="*70)
    print()
    print("このプログラムの目的:")
    print("• パン・チルトサーボを正確な中央位置（90度）に設定")
    print("• メカニカル部品組付け前の位置決め")
    print("• サーボホーン取り付け時の基準位置提供")
    print()
    print("重要な注意事項:")
    print("⚠ サーボホーン取り付け時は中央位置を維持してください")
    print("⚠ 組付け作業中はサーボに無理な力を加えないでください")
    print("⚠ 異常を感じた場合は即座にCtrl+Cで停止してください")
    print()


def main():
    """メイン処理"""
    print_header()
    
    # 初期位置調整オブジェクト作成
    position_setup = ServoPositionSetup()
    
    try:
        # シグナルハンドラー設定
        position_setup.setup_signal_handlers()
        
        # ハードウェア初期化
        if not position_setup.initialize_hardware():
            print("ハードウェア初期化に失敗しました。")
            sys.exit(1)
        
        # 最初に中央位置に移動
        position_setup.move_to_center_position()
        
        # 対話型メニュー開始
        position_setup.interactive_menu()
        
    except KeyboardInterrupt:
        print("\nプログラムが中断されました")
    except Exception as e:
        print(f"予期しないエラー: {e}")
    finally:
        # クリーンアップ
        position_setup.cleanup()
        
    print("\nサーボ初期位置調整プログラムを終了しました")


if __name__ == "__main__":
    main()