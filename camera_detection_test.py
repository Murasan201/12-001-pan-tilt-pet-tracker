#!/usr/bin/env python3
"""
犬猫検出サンプルプログラム - YOLOv8を用いたリアルタイム検出

このプログラムはRaspberry Pi カメラモジュール v3とYOLOv8モデルを使用して
犬と猫をリアルタイムで検出し、バウンディングボックスを表示します。

機能:
- カメラからのリアルタイム映像取得
- YOLOv8による犬猫検出
- バウンディングボックスとラベル表示
- FPS監視とパフォーマンス計測
- キー操作による終了・保存機能

操作方法:
- 'q'キー: プログラム終了
- 's'キー: 現在のフレームを画像保存
- 'ESC'キー: プログラム終了

安全注意事項:
- プログラム実行前にカメラが正しく接続されていることを確認してください
- モデルファイルのダウンロードには時間がかかる場合があります
- 処理能力に応じて解像度やモデルサイズを調整してください
"""

import cv2
import time
import argparse
import sys
import os
from datetime import datetime
from typing import List, Tuple, Optional

try:
    from ultralytics import YOLO
    import numpy as np
except ImportError as e:
    print(f"必要なライブラリがインストールされていません: {e}")
    print("以下のコマンドでインストールしてください:")
    print("pip install ultralytics opencv-python numpy")
    sys.exit(1)


class CameraDetector:
    """カメラ検出クラス - YOLOv8を用いた犬猫検出"""
    
    def __init__(self, model_path: str = "yolov8n.pt", confidence: float = 0.5, 
                 resolution: Tuple[int, int] = (640, 480), target_fps: int = 30):
        """
        コンストラクタ
        
        Args:
            model_path: YOLOv8モデルファイルのパス
            confidence: 検出信頼度の閾値
            resolution: カメラ解像度 (width, height)
            target_fps: 目標フレームレート
        """
        self.model_path = model_path
        self.confidence = confidence
        self.resolution = resolution
        self.target_fps = target_fps
        
        # カメラとモデルのインスタンス
        self.cap = None
        self.model = None
        
        # COCO dataset class IDs (YOLOv8で使用)
        self.DOG_CLASS_ID = 16    # dog
        self.CAT_CLASS_ID = 15    # cat
        
        # 表示色の設定 (BGR形式)
        self.colors = {
            self.DOG_CLASS_ID: (255, 0, 0),    # 青色
            self.CAT_CLASS_ID: (0, 255, 0),    # 緑色
        }
        
        # クラス名の設定
        self.class_names = {
            self.DOG_CLASS_ID: "Dog",
            self.CAT_CLASS_ID: "Cat"
        }
        
        # パフォーマンス監視用
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0.0
        self.processing_times = []
        
        # 検出統計
        self.dog_count = 0
        self.cat_count = 0
    
    def initialize_camera(self) -> bool:
        """
        カメラの初期化
        
        Returns:
            bool: 初期化成功時True、失敗時False
        """
        try:
            print("カメラを初期化中...")
            
            # カメラの初期化 (インデックス0: デフォルトカメラ)
            self.cap = cv2.VideoCapture(0)
            
            if not self.cap.isOpened():
                print("エラー: カメラを開くことができませんでした")
                return False
            
            # カメラ設定
            width, height = self.resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)
            
            # 実際の設定値を取得
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            print(f"カメラ設定完了:")
            print(f"  解像度: {actual_width}x{actual_height}")
            print(f"  FPS: {actual_fps}")
            
            return True
            
        except Exception as e:
            print(f"カメラ初期化中にエラーが発生しました: {e}")
            return False
    
    def load_model(self) -> bool:
        """
        YOLOv8モデルの読み込み
        
        Returns:
            bool: 読み込み成功時True、失敗時False
        """
        try:
            print(f"YOLOv8モデルを読み込み中: {self.model_path}")
            
            # モデルの読み込み（初回実行時は自動ダウンロード）
            self.model = YOLO(self.model_path)
            
            print("モデルの読み込みが完了しました")
            print(f"  モデル: {self.model_path}")
            print(f"  信頼度閾値: {self.confidence}")
            
            return True
            
        except Exception as e:
            print(f"モデル読み込み中にエラーが発生しました: {e}")
            return False
    
    def detect_pets(self, frame: np.ndarray) -> List[dict]:
        """
        フレーム内の犬猫検出
        
        Args:
            frame: 入力画像フレーム
            
        Returns:
            List[dict]: 検出結果のリスト
        """
        detection_start_time = time.time()
        
        try:
            # YOLOv8で推論実行
            results = self.model(frame, conf=self.confidence, verbose=False)
            
            detections = []
            
            # 検出結果の解析
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # クラスID、信頼度、座標の取得
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        
                        # 犬または猫の場合のみ処理
                        if class_id in [self.DOG_CLASS_ID, self.CAT_CLASS_ID]:
                            detections.append({
                                'class_id': class_id,
                                'confidence': confidence,
                                'bbox': (int(x1), int(y1), int(x2), int(y2)),
                                'class_name': self.class_names[class_id]
                            })
            
            # 処理時間の記録
            processing_time = time.time() - detection_start_time
            self.processing_times.append(processing_time)
            
            # 処理時間履歴の管理（最新100フレーム分のみ保持）
            if len(self.processing_times) > 100:
                self.processing_times.pop(0)
            
            return detections
            
        except Exception as e:
            print(f"検出処理中にエラーが発生しました: {e}")
            return []
    
    def draw_detections(self, frame: np.ndarray, detections: List[dict]) -> np.ndarray:
        """
        検出結果をフレームに描画
        
        Args:
            frame: 入力画像フレーム
            detections: 検出結果のリスト
            
        Returns:
            np.ndarray: 描画済みフレーム
        """
        # 検出数のカウント
        self.dog_count = 0
        self.cat_count = 0
        
        for detection in detections:
            class_id = detection['class_id']
            confidence = detection['confidence']
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class_name']
            
            # 検出数のカウント
            if class_id == self.DOG_CLASS_ID:
                self.dog_count += 1
            elif class_id == self.CAT_CLASS_ID:
                self.cat_count += 1
            
            # バウンディングボックスの描画
            color = self.colors[class_id]
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # ラベルテキストの作成
            label = f"{class_name} {confidence:.2f}"
            
            # ラベル背景の描画
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - text_height - 10), 
                         (x1 + text_width, y1), color, -1)
            
            # ラベルテキストの描画
            cv2.putText(frame, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame
    
    def draw_info(self, frame: np.ndarray) -> np.ndarray:
        """
        フレームに情報表示を描画
        
        Args:
            frame: 入力画像フレーム
            
        Returns:
            np.ndarray: 情報描画済みフレーム
        """
        height, width = frame.shape[:2]
        
        # FPS表示
        fps_text = f"FPS: {self.current_fps:.1f}"
        cv2.putText(frame, fps_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 検出数表示
        count_text = f"Dogs: {self.dog_count}, Cats: {self.cat_count}"
        cv2.putText(frame, count_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 平均処理時間表示
        if self.processing_times:
            avg_time = sum(self.processing_times) / len(self.processing_times)
            time_text = f"Avg Process Time: {avg_time*1000:.1f}ms"
            cv2.putText(frame, time_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 操作説明表示
        help_text = "Press 'q' or ESC to quit, 's' to save"
        text_size = cv2.getTextSize(help_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        cv2.putText(frame, help_text, (width - text_size[0] - 10, height - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def update_fps(self) -> None:
        """FPS計算の更新"""
        self.fps_counter += 1
        
        # 1秒ごとにFPSを更新
        if time.time() - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter / (time.time() - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = time.time()
    
    def save_frame(self, frame: np.ndarray) -> None:
        """現在のフレームを画像ファイルとして保存"""
        try:
            # 保存ディレクトリの作成
            save_dir = "captured_frames"
            os.makedirs(save_dir, exist_ok=True)
            
            # ファイル名の生成（タイムスタンプ付き）
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{save_dir}/detection_{timestamp}.jpg"
            
            # 画像の保存
            cv2.imwrite(filename, frame)
            print(f"フレームを保存しました: {filename}")
            
        except Exception as e:
            print(f"フレーム保存中にエラーが発生しました: {e}")
    
    def run(self) -> None:
        """メイン実行ループ"""
        print("\n" + "="*60)
        print("犬猫検出サンプルプログラム開始")
        print("="*60)
        print()
        print("操作方法:")
        print("  'q'キー: 終了")
        print("  's'キー: 現在のフレームを保存")
        print("  'ESC'キー: 終了")
        print()
        
        try:
            while True:
                # フレームの読み取り
                ret, frame = self.cap.read()
                if not ret:
                    print("エラー: フレームを読み取れませんでした")
                    break
                
                # 犬猫検出の実行
                detections = self.detect_pets(frame)
                
                # 検出結果の描画
                frame = self.draw_detections(frame, detections)
                
                # 情報表示の描画
                frame = self.draw_info(frame)
                
                # フレーム表示
                cv2.imshow('Pet Detection Test', frame)
                
                # FPS更新
                self.update_fps()
                
                # キー入力処理
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # 'q'キーまたはESCキー
                    print("\nユーザーによる終了要求を受信しました")
                    break
                elif key == ord('s'):  # 's'キー
                    self.save_frame(frame)
                
        except KeyboardInterrupt:
            print("\n\nCtrl+Cが押されました。プログラムを終了します...")
        
        except Exception as e:
            print(f"\n実行中にエラーが発生しました: {e}")
        
        finally:
            self.cleanup()
    
    def cleanup(self) -> None:
        """リソースのクリーンアップ"""
        try:
            print("\nリソースをクリーンアップ中...")
            
            if self.cap:
                self.cap.release()
                print("カメラリソースを解放しました")
            
            cv2.destroyAllWindows()
            print("OpenCVウィンドウを閉じました")
            
            # 統計情報の表示
            if self.processing_times:
                avg_time = sum(self.processing_times) / len(self.processing_times)
                max_time = max(self.processing_times)
                min_time = min(self.processing_times)
                
                print("\n" + "="*40)
                print("実行統計:")
                print(f"  平均処理時間: {avg_time*1000:.2f}ms")
                print(f"  最大処理時間: {max_time*1000:.2f}ms")
                print(f"  最小処理時間: {min_time*1000:.2f}ms")
                print(f"  最終FPS: {self.current_fps:.1f}")
                print("="*40)
            
            print("クリーンアップが完了しました")
            
        except Exception as e:
            print(f"クリーンアップ中にエラーが発生しました: {e}")


def parse_arguments() -> argparse.Namespace:
    """コマンドライン引数の解析"""
    parser = argparse.ArgumentParser(
        description="犬猫検出サンプルプログラム - YOLOv8リアルタイム検出",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  python camera_detection_test.py
  python camera_detection_test.py --model yolov8s.pt --confidence 0.6
  python camera_detection_test.py --resolution 1280 720 --fps 24
        """
    )
    
    parser.add_argument('--model', type=str, default='yolov8n.pt',
                       help='YOLOv8モデルファイルのパス (デフォルト: yolov8n.pt)')
    
    parser.add_argument('--confidence', type=float, default=0.5,
                       help='検出信頼度の閾値 (0.0-1.0, デフォルト: 0.5)')
    
    parser.add_argument('--resolution', type=int, nargs=2, default=[640, 480],
                       metavar=('WIDTH', 'HEIGHT'),
                       help='カメラ解像度 (デフォルト: 640 480)')
    
    parser.add_argument('--fps', type=int, default=30,
                       help='目標フレームレート (デフォルト: 30)')
    
    return parser.parse_args()


def main():
    """メイン関数"""
    print("=" * 60)
    print("犬猫検出サンプルプログラム")
    print("YOLOv8を用いたリアルタイム犬猫検出")
    print("=" * 60)
    
    # コマンドライン引数の解析
    args = parse_arguments()
    
    # 引数の検証
    if not (0.0 <= args.confidence <= 1.0):
        print("エラー: 信頼度は0.0から1.0の間で指定してください")
        sys.exit(1)
    
    if args.resolution[0] <= 0 or args.resolution[1] <= 0:
        print("エラー: 解像度は正の整数で指定してください")
        sys.exit(1)
    
    if args.fps <= 0:
        print("エラー: FPSは正の整数で指定してください")
        sys.exit(1)
    
    print(f"\n設定パラメータ:")
    print(f"  モデル: {args.model}")
    print(f"  信頼度閾値: {args.confidence}")
    print(f"  解像度: {args.resolution[0]}x{args.resolution[1]}")
    print(f"  目標FPS: {args.fps}")
    print()
    
    # CameraDetectorインスタンスの作成
    detector = CameraDetector(
        model_path=args.model,
        confidence=args.confidence,
        resolution=tuple(args.resolution),
        target_fps=args.fps
    )
    
    try:
        # カメラの初期化
        if not detector.initialize_camera():
            print("カメラの初期化に失敗しました。接続を確認してください。")
            sys.exit(1)
        
        # モデルの読み込み
        if not detector.load_model():
            print("モデルの読み込みに失敗しました。")
            sys.exit(1)
        
        print("初期化完了。検出を開始します...\n")
        
        # メイン実行ループ
        detector.run()
        
    except Exception as e:
        print(f"プログラム実行中にエラーが発生しました: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()