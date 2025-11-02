#!/usr/bin/env python3
"""
パン・チルトAI追跡カメラアプリ - メインプログラム

このプログラムは初心者向けのペット追跡システムです。
YOLOv8でペット（犬・猫）を検出し、Simple P制御でカメラを自動的に向けます。

主な機能:
🐕 犬・猫の自動検出（YOLOv8使用）
📹 リアルタイム映像表示
🎯 Simple P制御による自動追跡
🤖 サーボモータでのカメラ向き制御
📊 動作状況のリアルタイム表示

使用方法:
1. ハードウェアを接続してください
   - Raspberry Pi 5
   - カメラモジュール v3
   - PCA9685 サーボドライバ
   - SG90 サーボモータ x2

2. このプログラムを実行してください
   python3 main.py

3. カメラの前に犬や猫を映してください

4. 終了するには以下のいずれかの方法で:
   - 画面で 'q' キーを押す
   - Ctrl+C を入力

教育目的:
このプログラムは以下を学習できるよう設計されています:
- コンピュータビジョンの基礎
- Simple P制御の理解
- モジュラー設計の実践
- リアルタイムシステムの構築
"""

import sys
import signal
import logging
import argparse
import time
from datetime import datetime
from pathlib import Path

# プロジェクトルートをパスに追加
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from modules.tracking_coordinator import TrackingCoordinator


class PetTrackingApp:
    """
    ペット追跡アプリケーションのメインクラス
    
    初心者にも分かりやすいよう、設定と操作をシンプルにしています。
    """
    
    def __init__(self):
        """
        アプリケーション初期化
        
        内部で以下の処理を実行します:
        - ログシステムのセットアップ
        - シグナルハンドラの登録（Ctrl+C対応）
        """
        self.tracker = None
        self.is_running = False
        
        # ログ設定
        self.setup_logging()
        self.logger = logging.getLogger(__name__)
        
        # シグナルハンドラー設定
        self.setup_signal_handlers()
    
    def setup_logging(self):
        """ログ設定"""
        log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        logging.basicConfig(
            level=logging.INFO,
            format=log_format,
            handlers=[
                logging.StreamHandler(sys.stdout),
                logging.FileHandler(f'pet_tracking_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
            ]
        )
    
    def setup_signal_handlers(self):
        """シグナルハンドラー設定"""
        def signal_handler(signum, frame):
            self.logger.info(f"終了シグナル ({signum}) を受信しました")
            self.stop()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
    
    def print_welcome_message(self):
        """ウェルカムメッセージ表示"""
        print("=" * 60)
        print("🐕 パン・チルトAI追跡カメラアプリ 🐱")
        print("=" * 60)
        print()
        print("このアプリは以下の機能を提供します:")
        print("• YOLOv8による犬・猫の自動検出")
        print("• Simple P制御による自動追跡")
        print("• リアルタイム映像表示")
        print("• サーボモータでのカメラ制御")
        print()
        print("動作モード:")
        print("• STANDBY  : 待機モード（起動時）")
        print("• SCANNING : スキャンモード（対象を探している状態）")
        print("• TRACKING : 追跡モード（対象を追跡している状態）")
        print()
        print("操作方法:")
        print("• 画面で 'q' キー : 終了")
        print("• Ctrl+C        : 強制終了")
        print()
        print("=" * 60)
    
    def print_hardware_check(self):
        """ハードウェアチェック情報表示"""
        print("🔧 ハードウェア接続確認")
        print("-" * 30)
        print("以下のハードウェアが正しく接続されていることを確認してください:")
        print()
        print("1. Raspberry Pi 5")
        print("   - システム動作確認済み")
        print()
        print("2. カメラモジュール v3")
        print("   - カメラケーブル接続確認")
        print("   - カメラ有効化設定確認 (raspi-config)")
        print()
        print("3. PCA9685 サーボドライバ")
        print("   - I2C接続確認 (SDA: GPIO2, SCL: GPIO3)")
        print("   - 電源供給確認 (5V)")
        print()
        print("4. SG90 サーボモータ x2")
        print("   - パンサーボ  : PCA9685 チャンネル 0")
        print("   - チルトサーボ: PCA9685 チャンネル 1")
        print("   - 電源・信号線接続確認")
        print()
        
        # ユーザー確認
        try:
            response = input("ハードウェアの準備は完了していますか？ (y/N): ").strip().lower()
            if response not in ['y', 'yes']:
                print("ハードウェアの準備を完了してから再実行してください。")
                sys.exit(1)
        except KeyboardInterrupt:
            print("\n操作がキャンセルされました。")
            sys.exit(1)
        
        print("✓ ハードウェア確認完了\n")
    
    def create_tracker(self, args):
        """追跡システム作成"""
        try:
            self.tracker = TrackingCoordinator(
                camera_id=args.camera_id,
                image_width=args.width,
                image_height=args.height,
                detection_interval=args.interval,
                lost_target_timeout=args.timeout,
                show_display=args.display
            )
            
            self.logger.info("追跡システムを作成しました")
            return True
            
        except Exception as e:
            self.logger.error(f"追跡システム作成エラー: {e}")
            return False
    
    def run(self, args):
        """アプリケーション実行"""
        try:
            # ウェルカムメッセージ
            self.print_welcome_message()
            
            # ハードウェアチェック（対話モードの場合）
            if args.interactive:
                self.print_hardware_check()
            
            # 追跡システム作成
            if not self.create_tracker(args):
                self.logger.error("追跡システムの作成に失敗しました")
                return False
            
            self.logger.info("🚀 ペット追跡システムを開始します")
            print("📹 カメラの前に犬や猫を映してください...")
            print("🎯 検出すると自動的に追跡を開始します")
            print()
            
            # 追跡開始
            self.is_running = True
            self.tracker.start_tracking()
            
            # メインループ（ステータス表示）
            self._status_monitoring_loop()
            
            return True
            
        except KeyboardInterrupt:
            self.logger.info("ユーザーによる中断")
            return True
        except Exception as e:
            self.logger.error(f"アプリケーション実行エラー: {e}")
            return False
        finally:
            self.stop()
    
    def _status_monitoring_loop(self):
        """ステータス監視ループ"""
        last_status_time = 0
        status_interval = 10  # 10秒間隔で状態表示
        
        try:
            while self.is_running and self.tracker.is_running:
                current_time = time.time()
                
                # 定期的にシステム状態を表示
                if current_time - last_status_time > status_interval:
                    self._print_system_status()
                    last_status_time = current_time
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            self.logger.info("監視ループが中断されました")
    
    def _print_system_status(self):
        """システム状態表示"""
        try:
            if not self.tracker:
                return
            
            status = self.tracker.get_system_status()
            
            print("\n" + "="*50)
            print(f"📊 システム状態 ({datetime.now().strftime('%H:%M:%S')})")
            print("="*50)
            print(f"動作モード    : {status['mode'].upper()}")
            print(f"対象検出      : {'✓ YES' if status['target_detected'] else '✗ NO'}")
            if status['target_detected']:
                print(f"検出クラス    : {status['target_class']}")
                print(f"信頼度        : {status['target_confidence']:.2f}")
            print(f"カメラ角度    : Pan {status['pan_angle']:.1f}°, Tilt {status['tilt_angle']:.1f}°")
            print(f"最新補正      : ({status['correction_applied'][0]:.2f}, {status['correction_applied'][1]:.2f})")
            print(f"検出回数      : {status['total_detections']} 回")
            print(f"動作時間      : {status['tracking_duration']:.1f} 秒")
            print("="*50)
            
        except Exception as e:
            self.logger.error(f"ステータス表示エラー: {e}")
    
    def stop(self):
        """アプリケーション停止"""
        self.logger.info("アプリケーションを停止しています...")
        self.is_running = False
        
        if self.tracker:
            self.tracker.stop_tracking()
        
        print("\n" + "="*50)
        print("📈 最終統計情報")
        print("="*50)
        
        if self.tracker:
            final_status = self.tracker.get_system_status()
            print(f"総検出回数: {final_status['total_detections']} 回")
            print(f"総動作時間: {final_status['tracking_duration']:.1f} 秒")
            if final_status['total_detections'] > 0:
                avg_time = final_status['tracking_duration'] / final_status['total_detections']
                print(f"平均検出間隔: {avg_time:.1f} 秒")
        
        print("="*50)
        print("🙏 ご利用ありがとうございました！")
        print("="*50)


def parse_arguments():
    """
    コマンドライン引数解析
    
    Returns:
        argparse.Namespace: 解析された引数を含むオブジェクト
    """
    parser = argparse.ArgumentParser(
        description="パン・チルトAI追跡カメラアプリ",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  python3 main.py                    # 基本設定で実行
  python3 main.py --no-display       # 画面表示なしで実行  
  python3 main.py --camera-id 1      # カメラID 1を使用
  python3 main.py --width 1280 --height 720  # 高解像度で実行
        """
    )
    
    # カメラ設定
    parser.add_argument('--camera-id', type=int, default=0,
                      help='カメラID (デフォルト: 0)')
    parser.add_argument('--width', type=int, default=640,
                      help='画像幅 (デフォルト: 640)')
    parser.add_argument('--height', type=int, default=480,
                      help='画像高さ (デフォルト: 480)')
    
    # 動作設定
    parser.add_argument('--interval', type=float, default=0.5,
                      help='検出処理間隔（秒） (デフォルト: 0.5)')
    parser.add_argument('--timeout', type=float, default=5.0,
                      help='対象ロスト判定時間（秒） (デフォルト: 5.0)')
    
    # 表示設定
    parser.add_argument('--no-display', action='store_false', dest='display',
                      help='画面表示を無効にする')
    parser.add_argument('--no-interactive', action='store_false', dest='interactive',
                      help='対話モードを無効にする')
    
    # デバッグ設定
    parser.add_argument('--debug', action='store_true',
                      help='デバッグモードで実行')
    parser.add_argument('--log-level', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                      default='INFO', help='ログレベル (デフォルト: INFO)')
    
    return parser.parse_args()


def main():
    """
    メイン関数：アプリケーション実行エントリーポイント
    
    以下の処理を順序に実行します:
    1. コマンドライン引数を解析
    2. ログレベルを設定
    3. PetTrackingAppインスタンスを作成
    4. アプリケーション実行
    """
    # コマンドライン引数解析
    args = parse_arguments()
    
    # ログレベル設定
    if args.debug:
        args.log_level = 'DEBUG'
    
    # ログレベル適用
    logging.getLogger().setLevel(getattr(logging, args.log_level))
    
    # アプリケーション作成・実行
    app = PetTrackingApp()
    
    try:
        success = app.run(args)
        sys.exit(0 if success else 1)
        
    except Exception as e:
        print(f"予期しないエラーが発生しました: {e}")
        logging.error(f"予期しないエラー: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()