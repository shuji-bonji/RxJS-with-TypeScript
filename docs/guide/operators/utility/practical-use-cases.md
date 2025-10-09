---
description: tap、startWith、finalize、delay、timeoutなどユーティリティオペレーターの実践的な活用例を紹介します。ローディング状態管理、リアクティブフォーム検証、API呼び出し制御などUI開発で頻繁に使われるパターンを解説します。
---

# 実用的なユースケース

## ローディング状態の管理

`tap`、`finalize`などを使用して、ローディング状態を管理する例です。

```ts
import { of, throwError } from 'rxjs';
import { tap, delay, finalize, catchError } from 'rxjs/operators';

// UI要素
const loadingExample = document.createElement('div');
loadingExample.innerHTML = '<h3>API呼び出しとローディング状態の管理:</h3>';
document.body.appendChild(loadingExample);

// ローディングインジケータ
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = '読込中...';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.backgroundColor = '#e3f2fd';
loadingIndicator.style.borderRadius = '5px';
loadingIndicator.style.display = 'none';
loadingExample.appendChild(loadingIndicator);

// データ表示エリア
const dataContainer = document.createElement('div');
dataContainer.style.marginTop = '10px';
dataContainer.style.padding = '10px';
dataContainer.style.border = '1px solid #ddd';
dataContainer.style.borderRadius = '5px';
dataContainer.style.minHeight = '100px';
loadingExample.appendChild(dataContainer);

// 成功ボタン
const successButton = document.createElement('button');
successButton.textContent = '成功するリクエスト';
successButton.style.marginRight = '10px';
successButton.style.padding = '8px 16px';
loadingExample.insertBefore(successButton, loadingIndicator);

// 失敗ボタン
const failButton = document.createElement('button');
failButton.textContent = '失敗するリクエスト';
failButton.style.padding = '8px 16px';
loadingExample.insertBefore(failButton, loadingIndicator);

// 成功するAPIリクエストをシミュレート
function simulateSuccessRequest() {
  return of({
    id: 1,
    name: 'サンプルデータ',
    description: 'これはAPIから取得したデータです。'
  }).pipe(
    // リクエスト開始時にローディング表示
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // APIレイテンシをシミュレート
    delay(1500),
    // リクエスト完了時に常にローディング非表示
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// 失敗するAPIリクエストをシミュレート
function simulateFailRequest() {
  return throwError(() => new Error('APIリクエストに失敗しました')).pipe(
    // リクエスト開始時にローディング表示
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // APIレイテンシをシミュレート
    delay(1500),
    // エラーハンドリング
    catchError(error => {
      const errorElement = document.createElement('div');
      errorElement.textContent = `エラー: ${error.message}`;
      errorElement.style.color = 'red';
      dataContainer.appendChild(errorElement);
      
      return throwError(() => error);
    }),
    // リクエスト完了時に常にローディング非表示
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// 成功ボタンクリック
successButton.addEventListener('click', () => {
  // ボタン無効化
  successButton.disabled = true;
  failButton.disabled = true;
  
  simulateSuccessRequest().subscribe({
    next: data => {
      // データ表示
      const dataElement = document.createElement('div');
      dataElement.innerHTML = `
        <h4>${data.name}</h4>
        <p>${data.description}</p>
        <p><em>ID: ${data.id}</em></p>
      `;
      dataContainer.appendChild(dataElement);
    },
    error: err => {
      console.error('エラー:', err);
    },
    complete: () => {
      // ボタン再有効化
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});

// 失敗ボタンクリック
failButton.addEventListener('click', () => {
  // ボタン無効化
  successButton.disabled = true;
  failButton.disabled = true;
  
  simulateFailRequest().subscribe({
    next: () => {
      // 成功することはないが、念のため
    },
    error: () => {
      // エラーは既にcatchErrorで処理済み
      console.log('エラーハンドリング完了');
    },
    complete: () => {
      // ボタン再有効化
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});
```

## フォーム検証と送信

`startWith`、`tap`、`finalize`などを使用して、フォーム検証と送信処理を実装する例です。

```ts
import { fromEvent, combineLatest, of } from 'rxjs';
import { map, startWith, debounceTime, tap, finalize, catchError, delay } from 'rxjs/operators';

// フォームUI
const formExample = document.createElement('div');
formExample.innerHTML = '<h3>リアクティブフォームの例:</h3>';
document.body.appendChild(formExample);

// フォーム要素の作成
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formExample.appendChild(form);

// 名前入力フィールド
const nameLabel = document.createElement('label');
nameLabel.textContent = '名前: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.padding = '8px';
nameInput.style.width = '100%';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

const nameError = document.createElement('div');
nameError.style.color = 'red';
nameError.style.fontSize = '12px';
nameError.style.marginTop = '-10px';
nameError.style.marginBottom = '15px';
form.appendChild(nameError);

// メールアドレス入力フィールド
const emailLabel = document.createElement('label');
emailLabel.textContent = 'メールアドレス: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.padding = '8px';
emailInput.style.width = '100%';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

const emailError = document.createElement('div');
emailError.style.color = 'red';
emailError.style.fontSize = '12px';
emailError.style.marginTop = '-10px';
emailError.style.marginBottom = '15px';
form.appendChild(emailError);

// 送信ボタン
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = '送信';
submitButton.style.padding = '8px 16px';
submitButton.disabled = true; // 初期状態は無効
form.appendChild(submitButton);

// 結果表示エリア
const formResult = document.createElement('div');
formResult.style.marginTop = '20px';
formResult.style.padding = '10px';
formResult.style.border = '1px solid transparent';
formResult.style.borderRadius = '5px';
formResult.style.display = 'none';
formExample.appendChild(formResult);

// 名前入力の検証
const name$ = fromEvent(nameInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: '名前は必須です' };
    }
    if (value.length < 2) {
      return { value, valid: false, error: '名前は2文字以上入力してください' };
    }
    return { value, valid: true, error: null };
  })
);

// メール入力の検証
const emailRegex = /^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'メールアドレスは必須です' };
    }
    if (!emailRegex.test(value)) {
      return { value, valid: false, error: '有効なメールアドレスを入力してください' };
    }
    return { value, valid: true, error: null };
  })
);

// フォーム全体の検証状態を監視
combineLatest([name$, email$]).pipe(
  map(([nameState, emailState]) => {
    // フォーム全体が有効かどうか
    const isValid = nameState.valid && emailState.valid;
    
    // 検証エラーを表示
    nameError.textContent = nameState.error || '';
    emailError.textContent = emailState.error || '';
    
    return isValid;
  })
).subscribe(isValid => {
  // 送信ボタンの有効/無効を切り替え
  submitButton.disabled = !isValid;
});

// フォーム送信処理
fromEvent(form, 'submit').pipe(
  tap(event => {
    // フォームのデフォルト送信を防止
    event.preventDefault();
    
    // 送信中の状態にする
    submitButton.disabled = true;
    submitButton.textContent = '送信中...';
    
    // 結果表示エリアをリセット
    formResult.style.display = 'none';
  }),
  // フォームデータを取得
  map(() => ({
    name: nameInput.value.trim(),
    email: emailInput.value.trim()
  })),
  // APIリクエストのシミュレーション
  delay(1500),
  // 常に送信完了状態に戻す
  finalize(() => {
    submitButton.textContent = '送信';
    submitButton.disabled = false;
  }),
  // エラーハンドリング
  catchError(error => {
    formResult.textContent = `エラー: ${error.message}`;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#ffebee';
    formResult.style.borderColor = '#f44336';
    
    return of(null); // ストリームを続行
  })
).subscribe(data => {
  if (data) {
    // 送信成功
    formResult.innerHTML = `
      <div style="font-weight: bold;">送信成功!</div>
      <div>名前: ${data.name}</div>
      <div>メール: ${data.email}</div>
    `;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#e8f5e9';
    formResult.style.borderColor = '#4caf50';
    
    // フォームをリセット
    nameInput.value = '';
    emailInput.value = '';
  }
});
```

## ユーティリティオペレーターの選び方

| 目的 | オペレーター | 使用場面 |
|------|--------------|---------|
| サイドエフェクト実行 | `tap` | デバッグ、ログ出力、UIの更新など |
| 値の出力遅延 | `delay` | アニメーション、タイミング調整など |
| タイムアウト設定 | `timeout` | APIリクエスト、非同期処理のタイムアウト |
| 完了時の処理 | `finalize` | リソースのクリーンアップ、ローディング状態の解除 |
| 初期値の設定 | `startWith` | 状態の初期化、プレースホルダーの表示 |
| 配列への変換 | `toArray` | バッチ処理、全ての結果をまとめて処理 |
| エラー時の再試行 | `retry` | ネットワークリクエスト、一時的なエラーからの回復 |
| ストリームの繰り返し | `repeat` | ポーリング、定期的な処理 |

## まとめ

ユーティリティオペレーターは、RxJSでのプログラミングをより効率的かつ堅牢にするための重要なツールです。これらのオペレーターを適切に組み合わせることで、以下のような利点が得られます。

1. **デバッグの容易さ**: `tap`を使用することで、ストリームの中間状態を簡単に確認できます。
2. **エラー耐性**: `retry`、`timeout`、`catchError`を組み合わせることで、堅牢なエラー処理が可能になります。
3. **リソース管理**: `finalize`を使用することで、リソースの適切なクリーンアップが保証されます。
4. **UIの応答性向上**: `startWith`、`delay`などを使用して、ユーザー体験を向上させることができます。
5. **コードの可読性向上**: ユーティリティオペレーターを使用することで、副作用と純粋なデータ変換を明確に分離できます。

これらのオペレーターは、単独で使用するよりも、他のオペレーターと組み合わせて使用することで、その真価を発揮します。実際のアプリケーション開発では、複数のオペレーターを組み合わせて、複雑な非同期処理フローを管理することが一般的です。