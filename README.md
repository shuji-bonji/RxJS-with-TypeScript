# TypeScript で　RxJS
TypeScriptでの利用を前提とした、RxJS の学び場です。

## 環境構築

```sh
npm install
```

## ライブ編集反映（ホットリロードあり）
文書作成中に利用します。
```sh
npm run dev
```

## 本番ビルド

```sh
npm run build
```

## ビルド後の確認（Viteサーバー）
`npm run build` で生成されたHTMLをローカルで Viteサーバーを使って確認します。  
本番デプロイ前に確認したい時に利用します。
```sh
npm run preview
```

## ビルド後の確認（静的サーバー）
ビルド済みHTMLの静的配信	Nodeのserve-static的な仕組みで動作、軽量・高速だがホットリロードなし。  
生成されたHTMLを確認したい場合に利用します。（軽くて速い）

```sh
npm run serve
```

## デプロイ