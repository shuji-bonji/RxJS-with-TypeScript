# ハンズオン学習用の実行環境構築方法

このページでは、RxJSのコードをブラウザではなくローカル環境で即座に試せる開発テンプレート [`RxJS-with-TypeScript-Starter-Kit`](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit) の利用方法を紹介します。

## 特徴

- Vite + TypeScript + RxJS のシンプルな構成
- ホットリロード対応（`npm run dev` ですぐ動作確認）
- DOM操作もテストもできるローカル開発環境
- Vitestを使ったテスト駆動開発（TDD）にも対応可能

## 利用方法

以下のコマンドでクローン・セットアップできます。

```bash
git clone https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit.git
cd rxjs-with-typescript-starter-kit
npm install
npm run dev
```

ブラウザが自動起動し、`src/main.ts` に記述したコードが実行されます。

## 使用例

既存の`src/main.ts`を以下の様に書き換える。

```ts
// src/main.ts
import { interval } from 'rxjs';
import { take } from 'rxjs/operators';

const output = document.createElement('div');
document.body.appendChild(output);

interval(1000).pipe(take(5)).subscribe((val) => {
  const p = document.createElement('p');
  p.textContent = `カウント: ${val}`;
  output.appendChild(p);
});
```

## 推奨用途

- Observable / Subject / Operator の実験
- DOMと組み合わせたリアクティブUIの学習
- marbleテストの導入練習（`vitest` + `TestScheduler`）
- 自分のコードスニペット保管用のベース環境

## リンク

🔗 テンプレートはこちら → [RxJS-with-TypeScript-Starter-Kit](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit)  
詳しくは`README.md`を参照してください。