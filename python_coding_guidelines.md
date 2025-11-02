# Pythonコーディングガイドライン

本書およびサンプルプロジェクトで使用するPythonコードの開発ガイドラインです。

## 基本方針

本書は**プログラミング初心者向けの入門書**であるため、以下の優先順位でコードを記述します：

1. **わかりやすさ**（初心者が理解できること）
2. **実用性**（実際に動作し、役立つこと）
3. **品質**（保守しやすく、拡張可能なこと）

## コーディング規約

### PEP 8準拠

基本的に**PEP 8**（Pythonの公式スタイルガイド）に従います。

#### PEP 8の主要ルール

1. **インデント**: 4スペース
2. **行の長さ**: 最大79文字（ただし柔軟に対応）
3. **空行**: クラス・関数の前後に2行、関数内のセクション区切りに1行
4. **インポート**: 1行に1つ、順序は標準→サードパーティ→自作モジュール
5. **命名規則**:
   - 変数・関数: `snake_case`（スネークケース）
   - クラス: `PascalCase`（パスカルケース）
   - 定数: `UPPER_CASE`（大文字＋アンダースコア）

#### 本書での柔軟な運用

初心者向けであることを考慮し、以下のように運用します：

1. **基本的にPEP 8に従う**
   - 初心者に良い習慣を教えられる

2. **説明が必要な場合は簡潔に**
   - 「Pythonのスタイルガイド（PEP 8）では...」と記載

3. **厳密すぎない**
   - 79文字ルールは柔軟に（初心者には厳しい）
   - わかりやすさを最優先

### `if __name__ == "__main__":` の使用

すべての実行可能なPythonファイルに**必ず含める**こと。

#### 対象

以下の条件を満たすPythonファイル：

- ✅ 単独で実行可能なプログラム
- ✅ 20行以上のコード
- ✅ 完結した機能を持つ

#### 対象外

以下は含めなくてよい：

- ❌ 部分的なコード例（説明用の数行コード）
- ❌ インポート文だけの例
- ❌ 設定変更の例

#### 修正パターン

**修正前（悪い例）**

```python
#!/usr/bin/env python3
"""
サーボモーター基本制御
"""

import RPi.GPIO as GPIO
import time

# 設定
SERVO_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# PWM設定
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

# サーボを90度に移動
duty = 7.5
pwm.ChangeDutyCycle(duty)
time.sleep(1)

# クリーンアップ
pwm.stop()
GPIO.cleanup()
print("完了")
```

**修正後（良い例）**

```python
#!/usr/bin/env python3
"""
サーボモーター基本制御
"""

import RPi.GPIO as GPIO
import time

# 設定
SERVO_PIN = 18

def setup_servo():
    """サーボモーターを初期化"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, 50)
    pwm.start(0)
    return pwm

def move_servo(pwm, angle):
    """サーボを指定角度に移動"""
    duty = 2.5 + (angle / 180.0) * 10.0
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)

def cleanup_servo(pwm):
    """サーボをクリーンアップ"""
    pwm.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    # サーボを90度に移動
    pwm = setup_servo()

    try:
        move_servo(pwm, 90)
        print("サーボを90度に移動しました")
    finally:
        cleanup_servo()
        print("完了")
```

#### メリット

1. **再利用性**: 他のファイルからインポートして関数を使える
2. **テスト性**: 関数単位でテストしやすい
3. **保守性**: 処理が分離され、変更が容易
4. **プロフェッショナル**: 業界標準の書き方

### 関数化の指針

グローバルに実行されるコードは関数に移動します。

#### 関数の分類

1. **初期化関数**: `setup_*()`
   - ハードウェアの初期化
   - 設定の読み込み

2. **メイン処理関数**: `main()` または個別の関数
   - プログラムの主要な処理

3. **クリーンアップ関数**: `cleanup_*()`
   - リソースの解放
   - GPIO のクリーンアップ

#### try-finally の使用

リソースのクリーンアップは**必ず実行される**よう、`try-finally`を使用します。

```python
if __name__ == "__main__":
    device = setup_device()

    try:
        # メイン処理
        main_process(device)
    finally:
        # 確実にクリーンアップ
        cleanup_device(device)
```

### docstring（ドキュメント文字列）

すべての関数・クラスに**docstring**を記述します。

#### フォーマット

```python
def calculate_price(quantity, unit_price, tax_rate=0.1):
    """
    商品の合計金額を計算します。

    Args:
        quantity (int): 数量
        unit_price (float): 単価
        tax_rate (float): 消費税率（デフォルト: 0.1）

    Returns:
        float: 税込み合計金額
    """
    subtotal = quantity * unit_price
    total = subtotal * (1 + tax_rate)
    return total
```

#### 初心者向けの配慮

- **日本語で記述**
- **簡潔に**（1〜3行程度）
- **専門用語を避ける**

簡潔な例：

```python
def setup_servo():
    """サーボモーターを初期化して返す"""
    # 処理
    return pwm
```

### 命名規則

#### 変数・関数名

- **スネークケース**: `my_variable`, `calculate_total()`
- **意味のある名前**: `x` → `servo_angle`
- **動詞で始める**（関数）: `get_data()`, `calculate_price()`

#### クラス名

- **パスカルケース**: `MyClass`, `ServoController`
- **名詞**: `Device`, `DataProcessor`

#### 定数

- **大文字＋アンダースコア**: `MAX_SIZE = 100`, `DEFAULT_PORT = 8080`
- **ファイル先頭にまとめる**

```python
# 定数
SERVO_PIN = 18
PWM_FREQUENCY = 50
DEFAULT_ANGLE = 90

# 以下、関数定義など
```

### コメント

コメントの記述ルールについては、プロジェクトルートに`COMMENT_STYLE_GUIDE.md`が存在する場合は、そちらのルールに従ってください。

### インポート

#### 順序

1. **標準ライブラリ**
2. **サードパーティライブラリ**
3. **自作モジュール**

各グループの間に空行を入れます。

```python
# 標準ライブラリ
import sys
import time
from pathlib import Path

# サードパーティライブラリ
import RPi.GPIO as GPIO
from PIL import Image, ImageDraw

# 自作モジュール
from my_module import my_function
```

#### 1行に1つ

```python
# 良い例
import os
import sys

# 悪い例
import os, sys
```

#### fromインポートは具体的に

```python
# 良い例
from PIL import Image, ImageDraw, ImageFont

# 悪い例（何がインポートされるか不明確）
from PIL import *
```

### エラーハンドリング

#### 具体的な例外をキャッチ

```python
# 良い例
try:
    file = open("data.txt", "r")
except FileNotFoundError:
    print("ファイルが見つかりません")
except PermissionError:
    print("ファイルにアクセスできません")

# 悪い例（すべての例外をキャッチ）
try:
    file = open("data.txt", "r")
except:
    print("エラーが発生しました")
```

#### 初心者向けのエラーメッセージ

- **何が起きたか**を明確に
- **どう対処すべきか**を提示

```python
try:
    serial = i2c(port=1, address=I2C_ADDRESS)
except Exception as e:
    print(f"[I²C初期化]エラー: {e}")
    print("対処方法: I²C設定と配線を確認してください")
    print("ヒント: raspi-config で I²C を有効化")
    print("ヒント: i2cdetect -y 1 でデバイスを確認")
    sys.exit(1)
```

### ファイル構成

#### 推奨される順序

```python
#!/usr/bin/env python3
"""
ファイルの説明
"""

# インポート
import sys
import time

# 定数
MAX_SIZE = 100
DEFAULT_PORT = 8080

# 関数定義
def setup():
    """初期化"""
    pass

def main():
    """メイン処理"""
    pass

def cleanup():
    """クリーンアップ"""
    pass

# メイン実行部分
if __name__ == "__main__":
    setup()
    try:
        main()
    finally:
        cleanup()
```

## 書籍原稿での説明

### 初出時（第4章または第5章）

`if __name__ == "__main__":` について詳しく説明します。

```markdown
### プログラムの実行部分

最後に、`if __name__ == "__main__":` という行があります。これは「このファイルを直接実行したときだけ、以下のコードを実行する」という意味です。

この書き方をすることで、後で他のプログラムからこのファイルの関数を使いたい場合でも、意図せず実行されることを防げます。プロフェッショナルなPythonコードでは、この書き方が標準的に使われています。

今後のサンプルコードでも、この構文を使用していきます。
```

### 2回目以降（第6章〜）

簡潔に言及するのみ。

```markdown
プログラムの最後には、前章で説明した`if __name__ == "__main__":`を使用しています。これにより、このファイルを直接実行したときのみメイン処理が実行されます。
```

## チェックリスト

新しいPythonコードを作成する際は、以下を確認してください：

- [ ] PEP 8の基本ルールに従っている
- [ ] `if __name__ == "__main__":`が含まれている（実行可能なファイルの場合）
- [ ] グローバルな実行コードが関数に移動されている
- [ ] try-finallyでクリーンアップが確実に実行される
- [ ] すべての関数にdocstringがある
- [ ] 変数・関数名がわかりやすい（スネークケース）
- [ ] 定数が大文字で定義されている
- [ ] インポートが適切な順序で記述されている
- [ ] エラーメッセージが初心者にわかりやすい
- [ ] 適切なコメントが付いている

## 参考資料

- [PEP 8 -- Style Guide for Python Code](https://peps.python.org/pep-0008/)
- [PEP 20 -- The Zen of Python](https://peps.python.org/pep-0020/)
- [PEP 257 -- Docstring Conventions](https://peps.python.org/pep-0257/)

## 更新履歴

- 2025-11-02: 初版作成
