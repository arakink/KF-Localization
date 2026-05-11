# KF-Localization

このリポジトリは、以下のリポジトリをベースに内容を学習・整理したものです。

- https://github.com/ryuichiueda/LNPR_BOOK_CODES

元リポジトリのノートブック形式のカルマンフィルタ実装の一部を `.py` に書き直し、あわせて `docs/` に解説を追加しています。

元の著作物について:

- Copyright (C) 2019 Ryuichi UEDA
- MIT License のもとで公開されています

## セットアップ

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 実行

```bash
source .venv/bin/activate
python section_kalman_filter/kf1.py
```

## ドキュメント

- [`kf1.py` explanation](docs/kf1.md)
- [`kf2.py` explanation](docs/kf2.md)
- [`kf3.py` explanation](docs/kf3.md)
- [`kf4.py` explanation](docs/kf4.md)
