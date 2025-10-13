---
description: Vite、TypeScript、RxJSで構成された学習用開発テンプレートのセットアップ方法を紹介します。ホットリロード対応でブラウザ上でのコード実験やDOM操作、Vitestを使ったテスト駆動開発にも最適な環境です。
---

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
import { take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

interval(1000).pipe(take(5)).subscribe((val) => {
  const p = document.createElement('p');
  p.textContent = `カウント: ${val}`;
  output.appendChild(p);
});
```
### localhostにアクセス
以下のように`http://localhost:5174/`と表記されるので、こちらにアクセスして結果を確認しましょう。  
`console.log()`の結果の確認にはディベロッパーツールのコンソールで確認してください。

```sh
% npm run dev

> rxjs-with-typescript-starter-kit@0.0.0 dev
> vite

Port 5173 is in use, trying another one...

  VITE v6.3.1  ready in 107 ms

  ➜  Local:   http://localhost:5174/
  ➜  Network: use --host to expose
  ➜  press h + enter to show help
```

## 推奨用途

- Observable / Subject / Operator の実験
- DOMと組み合わせたリアクティブUIの学習
- marbleテストの導入練習（`vitest` + `TestScheduler`）
- 自分のコードスニペット保管用のベース環境

## リンク

🔗 テンプレートはこちら → [RxJS-with-TypeScript-Starter-Kit](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit)  
詳しくは`README.md`を参照してください。