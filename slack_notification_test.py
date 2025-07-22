#!/usr/bin/env python3
"""
Slack通知サンプルプログラム - カメラ撮影画像のSlack送信

このプログラムはRaspberry Pi カメラモジュール v3で撮影した画像を
Slack WebhookのURLを使用してSlackチャンネルに送信します。

機能:
- 環境変数からSlack Webhook URLの読み込み
- カメラによる画像撮影
- 撮影画像のローカル保存
- Slack Webhookを用いた画像送信
- エラーハンドリングと再送処理

設定方法:
1. .envファイルを作成
2. SLACK_WEBHOOK_URL=your_webhook_url を記載
3. プログラム実行

安全注意事項:
- .envファイルはGitにコミットしないでください
- Webhook URLは機密情報として適切に管理してください
- カメラが正しく接続されていることを確認してください
"""

import cv2
import os
import sys
import time
import argparse
import requests
from datetime import datetime
from typing import Optional, Tuple
from pathlib import Path

try:
    from dotenv import load_dotenv
except ImportError as e:
    print(f"必要なライブラリがインストールされていません: {e}")
    print("以下のコマンドでインストールしてください:")
    print("pip install python-dotenv opencv-python requests")
    sys.exit(1)


class SlackNotifier:
    """Slack通知クラス - カメラ撮影画像のSlack送信"""
    
    def __init__(self, bot_token: str, channel: str, message: str = "Camera capture from Raspberry Pi",
                 resolution: Tuple[int, int] = (1280, 720), save_local: bool = True):
        """
        コンストラクタ
        
        Args:
            bot_token: Slack Bot Token
            channel: 送信先チャンネル（#general や C1234567890）
            message: 送信メッセージ
            resolution: カメラ解像度 (width, height)
            save_local: ローカル保存フラグ
        """
        self.bot_token = bot_token
        self.channel = channel
        self.message = message
        self.resolution = resolution
        self.save_local = save_local
        
        # カメラインスタンス
        self.cap = None
        
        # 通信設定
        self.timeout = 30.0      # タイムアウト時間（秒）
        self.max_retries = 3     # 最大再送回数
        self.retry_delay = 2.0   # 再送間隔（秒）
        
        # ファイル保存設定
        self.save_dir = "slack_captures"
    
    def initialize_camera(self) -> bool:
        """
        カメラの初期化
        
        Returns:
            bool: 初期化成功時True、失敗時False
        """
        try:
            print("カメラを初期化中...")
            
            # カメラの初期化
            self.cap = cv2.VideoCapture(0)
            
            if not self.cap.isOpened():
                print("エラー: カメラを開くことができませんでした")
                return False
            
            # カメラ設定
            width, height = self.resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            
            # 実際の設定値を取得
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"カメラ初期化完了:")
            print(f"  解像度: {actual_width}x{actual_height}")
            
            return True
            
        except Exception as e:
            print(f"カメラ初期化中にエラーが発生しました: {e}")
            return False
    
    def capture_image(self) -> Optional[str]:
        """
        カメラで画像撮影とファイル保存
        
        Returns:
            Optional[str]: 保存されたファイルパス（失敗時はNone）
        """
        try:
            print("画像を撮影中...")
            
            # フレームキャプチャ
            ret, frame = self.cap.read()
            if not ret:
                print("エラー: フレームをキャプチャできませんでした")
                return None
            
            # 保存ディレクトリの作成
            if self.save_local:
                os.makedirs(self.save_dir, exist_ok=True)
            
            # ファイル名の生成（タイムスタンプ付き）
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"slack_capture_{timestamp}.jpg"
            
            if self.save_local:
                filepath = os.path.join(self.save_dir, filename)
            else:
                # 一時ファイルとして保存
                filepath = f"/tmp/{filename}"
            
            # JPEG形式で画像保存
            # 高品質設定（品質90%）
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 90]
            success = cv2.imwrite(filepath, frame, encode_params)
            
            if not success:
                print("エラー: 画像の保存に失敗しました")
                return None
            
            print(f"画像撮影完了: {filepath}")
            return filepath
            
        except Exception as e:
            print(f"画像撮影中にエラーが発生しました: {e}")
            return None
    
    def send_to_slack(self, image_path: str) -> bool:
        """
        Slackに画像を送信（Files API使用）
        
        Args:
            image_path: 送信する画像ファイルのパス
            
        Returns:
            bool: 送信成功時True、失敗時False
        """
        for attempt in range(1, self.max_retries + 1):
            try:
                print(f"Slackに送信中... (試行 {attempt}/{self.max_retries})")
                
                # ファイルの存在確認
                if not os.path.exists(image_path):
                    print(f"エラー: ファイルが見つかりません: {image_path}")
                    return False
                
                # メッセージペイロードの準備
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                file_size = os.path.getsize(image_path)
                detailed_message = (
                    f"{self.message}\n"
                    f"📅 撮影時刻: {timestamp}\n"
                    f"📁 ファイルサイズ: {file_size:,} bytes\n"
                    f"📸 解像度: {self.resolution[0]}x{self.resolution[1]}"
                )
                
                # Slack Files API エンドポイント
                url = "https://slack.com/api/files.upload"
                
                # ヘッダー設定
                headers = {
                    'Authorization': f'Bearer {self.bot_token}'
                }
                
                # ファイルアップロードのデータ
                with open(image_path, 'rb') as file:
                    files = {
                        'file': (os.path.basename(image_path), file, 'image/jpeg')
                    }
                    data = {
                        'channels': self.channel,
                        'initial_comment': detailed_message,
                        'title': 'Pet Tracker Camera Capture',
                        'filename': os.path.basename(image_path)
                    }
                    
                    # Files API経由でアップロード
                    response = requests.post(
                        url,
                        headers=headers,
                        files=files,
                        data=data,
                        timeout=self.timeout
                    )
                
                # レスポンスの確認
                if response.status_code == 200:
                    try:
                        import json
                        response_data = json.loads(response.text)
                        if response_data.get('ok'):
                            print("✅ Slackへの画像送信が完了しました")
                            return True
                        else:
                            error_msg = response_data.get('error', '不明なエラー')
                            print(f"❌ Slack API エラー: {error_msg}")
                            return False
                    except json.JSONDecodeError:
                        print(f"❌ レスポンス解析エラー: {response.text}")
                        return False
                else:
                    print(f"❌ HTTP エラー: {response.status_code} - {response.text}")
                    
                    # サーバーエラー（5xx）の場合は再試行
                    if response.status_code >= 500:
                        if attempt < self.max_retries:
                            print(f"サーバーエラーのため {self.retry_delay} 秒後に再試行します...")
                            time.sleep(self.retry_delay)
                            continue
                    
                    return False
                    
            except requests.exceptions.Timeout:
                print(f"❌ タイムアウトエラー ({self.timeout}秒)")
                if attempt < self.max_retries:
                    print(f"{self.retry_delay} 秒後に再試行します...")
                    time.sleep(self.retry_delay)
                    continue
                return False
                
            except requests.exceptions.ConnectionError:
                print("❌ 接続エラー: インターネット接続を確認してください")
                if attempt < self.max_retries:
                    print(f"{self.retry_delay} 秒後に再試行します...")
                    time.sleep(self.retry_delay)
                    continue
                return False
                
            except Exception as e:
                print(f"❌ 送信中にエラーが発生しました: {e}")
                if attempt < self.max_retries:
                    print(f"{self.retry_delay} 秒後に再試行します...")
                    time.sleep(self.retry_delay)
                    continue
                return False
        
        print(f"❌ {self.max_retries} 回の試行すべてが失敗しました")
        return False
    
    def run(self, test_mode: bool = False, sample_image_path: Optional[str] = None) -> bool:
        """
        メイン実行処理
        
        Args:
            test_mode: テストモード（Slack送信なし）
            sample_image_path: サンプル画像のパス（カメラが利用できない場合）
            
        Returns:
            bool: 実行成功時True、失敗時False
        """
        try:
            print("\n" + "="*60)
            print("Slack通知サンプルプログラム開始")
            print("="*60)
            print()
            
            # サンプル画像使用またはカメラ撮影
            if sample_image_path:
                # サンプル画像を使用
                print(f"サンプル画像を使用: {sample_image_path}")
                if not os.path.exists(sample_image_path):
                    print(f"エラー: サンプル画像が見つかりません: {sample_image_path}")
                    return False
                image_path = sample_image_path
            else:
                # カメラ初期化
                if not self.initialize_camera():
                    return False
                
                # 画像撮影
                image_path = self.capture_image()
                if not image_path:
                    return False
            
            # テストモードの場合はSlack送信をスキップ
            if test_mode:
                print("🧪 テストモード: Slack送信をスキップします")
                print(f"📁 撮影画像: {image_path}")
                return True
            
            # Slack送信
            success = self.send_to_slack(image_path)
            
            # 一時ファイルのクリーンアップ（ローカル保存なしの場合）
            if not self.save_local and image_path.startswith('/tmp/'):
                try:
                    os.remove(image_path)
                    print(f"一時ファイルを削除しました: {image_path}")
                except Exception as e:
                    print(f"一時ファイル削除中にエラー: {e}")
            
            return success
            
        except Exception as e:
            print(f"実行中にエラーが発生しました: {e}")
            return False
    
    def cleanup(self) -> None:
        """リソースのクリーンアップ"""
        try:
            if self.cap:
                self.cap.release()
                print("カメラリソースを解放しました")
        except Exception as e:
            print(f"クリーンアップ中にエラーが発生しました: {e}")


def load_environment_variables() -> Tuple[Optional[str], Optional[str]]:
    """
    環境変数の読み込み
    
    Returns:
        Tuple[Optional[str], Optional[str]]: (Bot Token, Channel)（読み込み失敗時はNone）
    """
    try:
        # .envファイルの読み込み
        env_path = Path('.env')
        if env_path.exists():
            load_dotenv(env_path)
            print(f"✅ .envファイルを読み込みました: {env_path.absolute()}")
        else:
            print("⚠️  .envファイルが見つかりません。環境変数から読み込みます。")
        
        # Bot Tokenの取得
        bot_token = os.getenv('SLACK_BOT_TOKEN')
        
        if not bot_token:
            print("❌ SLACK_BOT_TOKEN が設定されていません")
            print()
            print("設定方法:")
            print("1. .envファイルを作成")
            print("2. 以下の内容を記載:")
            print("   SLACK_BOT_TOKEN=xoxb-xxxxxxxxxxxxx")
            print("   SLACK_CHANNEL=#general")
            print("3. プログラムを再実行")
            return None, None
        
        # Channelの取得
        channel = os.getenv('SLACK_CHANNEL', '#general')
        
        # Bot Tokenの基本的な妥当性チェック
        if not bot_token.startswith('xoxb-'):
            print("❌ 無効なSlack Bot Tokenです")
            print("正しい形式: xoxb-xxxxxxxxxxxxx")
            return None, None
        
        print("✅ Slack Bot Tokenを読み込みました")
        print(f"✅ 送信先チャンネル: {channel}")
        return bot_token, channel
        
    except Exception as e:
        print(f"環境変数読み込み中にエラーが発生しました: {e}")
        return None, None


def create_gitignore_if_needed() -> None:
    """必要に応じて.gitignoreファイルを作成・更新"""
    gitignore_path = Path('.gitignore')
    env_entry = '.env'
    
    try:
        # 既存の.gitignoreをチェック
        if gitignore_path.exists():
            content = gitignore_path.read_text()
            if env_entry in content:
                return  # 既に.envが記載済み
        
        # .gitignoreに.envを追加
        with open(gitignore_path, 'a') as f:
            if gitignore_path.exists() and not content.endswith('\n'):
                f.write('\n')
            f.write('# Environment variables\n')
            f.write('.env\n')
            f.write('\n# Slack captures\n')
            f.write('slack_captures/\n')
        
        print("✅ .gitignoreファイルを更新しました (.env を追加)")
        
    except Exception as e:
        print(f"⚠️  .gitignore更新中にエラー: {e}")


def parse_arguments() -> argparse.Namespace:
    """コマンドライン引数の解析"""
    parser = argparse.ArgumentParser(
        description="Slack通知サンプルプログラム - カメラ撮影画像のSlack送信",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  python slack_notification_test.py
  python slack_notification_test.py --message "ペットを発見しました！"
  python slack_notification_test.py --resolution 1920 1080 --no-save-local
  python slack_notification_test.py --test-mode
        """
    )
    
    parser.add_argument('--message', type=str, 
                       default='Camera capture from Raspberry Pi',
                       help='Slackに送信するメッセージ')
    
    parser.add_argument('--resolution', type=int, nargs=2, default=[1280, 720],
                       metavar=('WIDTH', 'HEIGHT'),
                       help='カメラ解像度 (デフォルト: 1280 720)')
    
    parser.add_argument('--no-save-local', action='store_true',
                       help='ローカルファイル保存を無効化')
    
    parser.add_argument('--test-mode', action='store_true',
                       help='テストモード（Slack送信なし）')
    
    parser.add_argument('--sample-image', type=str,
                       help='サンプル画像のパス（カメラの代わりに使用）')
    
    return parser.parse_args()


def main():
    """メイン関数"""
    print("=" * 60)
    print("Slack通知サンプルプログラム")
    print("カメラ撮影画像のSlack送信テスト")
    print("=" * 60)
    
    # コマンドライン引数の解析
    args = parse_arguments()
    
    # 引数の検証
    if args.resolution[0] <= 0 or args.resolution[1] <= 0:
        print("エラー: 解像度は正の整数で指定してください")
        sys.exit(1)
    
    save_local = not args.no_save_local
    
    print(f"\n設定パラメータ:")
    print(f"  メッセージ: {args.message}")
    print(f"  解像度: {args.resolution[0]}x{args.resolution[1]}")
    print(f"  ローカル保存: {'有効' if save_local else '無効'}")
    print(f"  テストモード: {'有効' if args.test_mode else '無効'}")
    print()
    
    # .gitignoreの確認・作成
    create_gitignore_if_needed()
    
    # 環境変数の読み込み
    if not args.test_mode:
        bot_token, channel = load_environment_variables()
        if not bot_token or not channel:
            sys.exit(1)
    else:
        bot_token = "test_mode"  # テストモード用のダミートークン
        channel = "#test"
    
    # SlackNotifierインスタンスの作成
    notifier = SlackNotifier(
        bot_token=bot_token,
        channel=channel,
        message=args.message,
        resolution=tuple(args.resolution),
        save_local=save_local
    )
    
    try:
        # メイン処理の実行
        success = notifier.run(test_mode=args.test_mode, sample_image_path=args.sample_image)
        
        if success:
            print("\n✅ すべての処理が正常に完了しました！")
            sys.exit(0)
        else:
            print("\n❌ 処理中にエラーが発生しました")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n\nCtrl+Cが押されました。プログラムを終了します...")
        sys.exit(0)
        
    except Exception as e:
        print(f"\nプログラム実行中にエラーが発生しました: {e}")
        sys.exit(1)
        
    finally:
        notifier.cleanup()


if __name__ == "__main__":
    main()