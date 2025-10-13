---
description: debounceTimeやthrottleTimeなどのフィルタリングオペレーターを活用したリアルタイム検索や無限スクロールの実装例を紹介します。
---

# 実践的なユースケース

## ユーザー入力のリアルタイム検索フィルタリング

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  debounceTime,
  distinctUntilChanged,
  filter,
} from 'rxjs';

// UI構築
const searchInput = document.createElement('input');
searchInput.placeholder = '検索語を入力（3文字以上）';
document.body.appendChild(searchInput);

const resultsContainer = document.createElement('div');
document.body.appendChild(resultsContainer);

// イベントストリーム
fromEvent(searchInput, 'input')
  .pipe(
    map((event) => (event.target as HTMLInputElement).value.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((term) => term.length >= 3)
  )
  .subscribe((searchTerm) => {
    resultsContainer.innerHTML = `「${searchTerm}」の検索を開始...`;
  });

```

- **300ms間隔で確定入力のみ処理**します。
- **3文字以上**入力された場合にのみ検索が実行されます。
- **同じ単語**の連続入力は無視されます。
 

## 無限スクロールのシミュレーション

```ts
import { fromEvent } from 'rxjs';
import {
  map,
  filter,
  throttleTime,
  distinctUntilChanged,
  scan,
} from 'rxjs';

// UI構築
const scrollArea = document.createElement('div');
scrollArea.style.height = '200px';
scrollArea.style.overflow = 'auto';
scrollArea.style.border = '1px solid #ccc';
document.body.appendChild(scrollArea);

const itemsList = document.createElement('div');
scrollArea.appendChild(itemsList);

// 初期データ追加
function addItems(page: number) {
  for (let i = 1; i <= 10; i++) {
    const item = document.createElement('div');
    item.textContent = `アイテム ${(page - 1) * 10 + i}`;
    itemsList.appendChild(item);
  }
}
addItems(1);

// スクロールイベントストリーム
fromEvent(scrollArea, 'scroll')
  .pipe(
    throttleTime(200),
    map(() => ({
      scrollTop: scrollArea.scrollTop,
      scrollHeight: scrollArea.scrollHeight,
      clientHeight: scrollArea.clientHeight,
    })),
    map(
      ({ scrollTop, scrollHeight, clientHeight }) =>
        (scrollTop + clientHeight) / scrollHeight
    ),
    distinctUntilChanged(),
    filter((ratio) => ratio > 0.8),
    scan((page) => page + 1, 1),
    filter((page) => page <= 5)
  )
  .subscribe((page) => {
    addItems(page);
  });

```

- スクロール位置が**80%以上**になったら次のアイテムをロードします。
- **5ページ分**まで自動ロードします。
- **スクロールイベント**を**200msごと**に間引き処理します。
 

## フィルタリングオペレーターの選び方まとめ

| やりたいこと | オペレーター | 説明 |
|:---|:---|:---|
| 条件に合うデータだけ通す | `filter` | 最も基本的なフィルタリング |
| 最初の数件だけ取得したい | `take`, `first` | 取り込み件数制限 |
| 入力確定まで待機したい | `debounceTime` | フォーム入力に最適 |
| 一定間隔でのみ処理したい | `throttleTime` | スクロールやリサイズなどに適用 |
| 連続する同じ値を無視したい | `distinctUntilChanged` | 同一データの無駄な再処理を防止 |
 

## まとめ

- フィルタリングオペレーターは、データストリームの制御に欠かせない。
- 単体で使うだけでなく、**組み合わせ**ることでさらに強力になる。
- イベントドリブンアプリケーションやUI開発において、**効率化とパフォーマンス向上**に直結する。
 
