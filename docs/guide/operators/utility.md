# ユーティリティオペレーター

ユーティリティオペレーターは、RxJSの中でも特に便利なヘルパー機能を提供するオペレーター群です。これらは直接データを変換するのではなく、サイドエフェクトの処理、デバッグ、タイミング制御など、ストリームの操作をより簡単かつ効果的に行うための補助的な役割を果たします。

## tap - サイドエフェクトの実行

`tap`オペレーターは、「ストリームを変更せずにサイドエフェクト（副作用）を実行する」ために使用します。
ログの出力、デバッグ、または値に影響を与えないその他の操作に最適です。

```ts
import { of } from 'rxjs';
import { tap, map } from 'rxjs/operators';

// ログ出力用の要素
const logOutput = document.createElement('div');
document.body.appendChild(logOutput);

// 値のシーケンス
of(1, 2, 3, 4, 5).pipe(
  tap(val => {
    console.log(`元の値: ${val}`);
    
    // UIにログを追加
    const logEntry = document.createElement('div');
    logEntry.textContent = `tap: 値 ${val} が通過`;
    logEntry.style.color = '#666';
    logOutput.appendChild(logEntry);
  }),
  map(val => val * 10),
  tap(val => {
    console.log(`変換後の値: ${val}`);
    
    // UIにログを追加
    const logEntry = document.createElement('div');
    logEntry.textContent = `tap: 変換後の値 ${val}`;
    logEntry.style.color = '#090';
    logOutput.appendChild(logEntry);
  })
).subscribe(val => {
  // 最終結果をUIに表示
  const resultItem = document.createElement('div');
  resultItem.textContent = `結果: ${val}`;
  resultItem.style.fontWeight = 'bold';
  logOutput.appendChild(resultItem);
});
```

`tap`はストリームの中間状態を確認したり、デバッグしたりするのに非常に便利です。  
「**値の流れを変更せず**に、必要な副作用（ログ出力やUIの更新など）を実行」できます。

## delay - 値の遅延

`delay`オペレーターは、ストリーム内の各値の発行を指定した時間だけ遅延させます。

```ts
import { of } from 'rxjs';
import { delay, tap } from 'rxjs/operators';

// 出力表示エリア
const delayOutput = document.createElement('div');
delayOutput.innerHTML = '<h3>delay の例:</h3>';
document.body.appendChild(delayOutput);

// 現在時刻を表示する関数
function addTimeLog(message: string) {
  const now = new Date();
  const time = now.toLocaleTimeString('ja-JP', { hour12: false }) + 
    '.' + now.getMilliseconds().toString().padStart(3, '0');
  
  const logItem = document.createElement('div');
  logItem.textContent = `${time}: ${message}`;
  delayOutput.appendChild(logItem);
}

// 開始時刻を記録
addTimeLog('開始');

// 値のシーケンス
of('A', 'B', 'C').pipe(
  tap(val => addTimeLog(`値 ${val} が発行される前`)),
  delay(1000), // 1秒遅延
  tap(val => addTimeLog(`値 ${val} が1秒後に発行された`))
).subscribe();
```

`delay`は、アニメーション効果の実装や、APIリクエストのスロットリング、ユーザーフィードバックの遅延表示などに役立ちます。

## timeout - タイムアウト設定

`timeout`オペレーターは、指定した時間内に値が発行されない場合にエラーを発生させます。APIリクエストなどの操作にタイムアウトを設定するのに便利です。

```ts
import { interval, of } from 'rxjs';
import { timeout, catchError, take } from 'rxjs/operators';

// 出力表示エリア
const timeoutOutput = document.createElement('div');
timeoutOutput.innerHTML = '<h3>timeout の例:</h3>';
document.body.appendChild(timeoutOutput);

// タイムアウトの成功例
const normalStream$ = interval(500).pipe(take(5));

const timeoutSuccess = document.createElement('div');
timeoutSuccess.innerHTML = '<h4>正常なストリーム (タイムアウトなし):</h4>';
timeoutOutput.appendChild(timeoutSuccess);

normalStream$.pipe(
  timeout(1000), // 1秒のタイムアウト
  catchError(err => {
    const errorMsg = document.createElement('div');
    errorMsg.textContent = `エラー: ${err.message}`;
    errorMsg.style.color = 'red';
    timeoutSuccess.appendChild(errorMsg);
    return of('エラー後のフォールバック値');
  })
).subscribe(val => {
  const item = document.createElement('div');
  item.textContent = `値: ${val}`;
  timeoutSuccess.appendChild(item);
});

// タイムアウトのエラー例
const slowStream$ = interval(1500).pipe(take(5)); // 1.5秒ごとに値を発行

const timeoutError = document.createElement('div');
timeoutError.innerHTML = '<h4>遅いストリーム (タイムアウト発生):</h4>';
timeoutOutput.appendChild(timeoutError);

slowStream$.pipe(
  timeout(1000), // 1秒のタイムアウト
  catchError(err => {
    const errorMsg = document.createElement('div');
    errorMsg.textContent = `エラー: ${err.message}`;
    errorMsg.style.color = 'red';
    timeoutError.appendChild(errorMsg);
    return of('タイムアウト後のフォールバック値');
  })
).subscribe(val => {
  const item = document.createElement('div');
  item.textContent = `値: ${val}`;
  timeoutError.appendChild(item);
});
```

`timeout`は、ネットワークリクエストや非同期操作が予想以上に時間がかかる場合にエラーを発生させ、代替アクションを実行するのに役立ちます。特にAPIリクエストやユーザーインタラクションのタイムアウト処理に便利です。

## finalize - 完了時の処理

`finalize`オペレーターは、ストリームが完了、エラー、またはアンサブスクライブされたときに実行される関数を指定します。リソースのクリーンアップや最終処理に最適です。

```ts
import { interval } from 'rxjs';
import { take, finalize, tap } from 'rxjs/operators';

// 出力表示エリア
const finalizeOutput = document.createElement('div');
finalizeOutput.innerHTML = '<h3>finalize の例:</h3>';
document.body.appendChild(finalizeOutput);

// ローディングインジケータ
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'データ読み込み中...';
loadingIndicator.style.backgroundColor = '#e0f7fa';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.borderRadius = '5px';
finalizeOutput.appendChild(loadingIndicator);

// 進捗状況表示
const progressContainer = document.createElement('div');
progressContainer.style.marginTop = '10px';
finalizeOutput.appendChild(progressContainer);

// 完了メッセージ用の要素
const completionMessage = document.createElement('div');
completionMessage.style.marginTop = '10px';
completionMessage.style.fontWeight = 'bold';
finalizeOutput.appendChild(completionMessage);

// データ取得のシミュレーション
interval(500).pipe(
  take(5), // 5つの値を取得
  tap(val => {
    // 進捗状況表示
    const progressItem = document.createElement('div');
    progressItem.textContent = `アイテム ${val + 1} を処理中...`;
    progressContainer.appendChild(progressItem);
  }),
  finalize(() => {
    // ローディングインジケータを非表示
    loadingIndicator.style.display = 'none';
    
    // 完了メッセージ表示
    completionMessage.textContent = '処理が完了しました！';
    completionMessage.style.color = 'green';
  })
).subscribe({
  complete: () => {
    const successMsg = document.createElement('div');
    successMsg.textContent = 'すべてのデータが正常に読み込まれました。';
    completionMessage.appendChild(successMsg);
  }
});
```

`finalize`は、ストリームの終了時に必ず実行したい処理（リソースの解放、ローディングインジケータの非表示など）に最適です。エラーが発生しても、ストリームが正常に完了しても、または手動でアンサブスクライブしても、必ず実行されるため、クリーンアップ処理に非常に便利です。

## repeat - ストリームの繰り返し

`repeat`オペレーターは、ソースObservableが完了した後に、指定した回数だけストリーム全体を繰り返します。

```ts
import { of } from 'rxjs';
import { repeat, tap } from 'rxjs/operators';

// 出力表示エリア
const repeatOutput = document.createElement('div');
repeatOutput.innerHTML = '<h3>repeat の例:</h3>';
document.body.appendChild(repeatOutput);

// 繰り返し回数の表示
let repeatCount = 0;
const repeatCountDisplay = document.createElement('div');
repeatCountDisplay.textContent = `繰り返し回数: ${repeatCount}`;
repeatCountDisplay.style.fontWeight = 'bold';
repeatOutput.appendChild(repeatCountDisplay);

// 値の出力エリア
const valuesOutput = document.createElement('div');
valuesOutput.style.marginTop = '10px';
valuesOutput.style.padding = '10px';
valuesOutput.style.border = '1px solid #ddd';
valuesOutput.style.maxHeight = '200px';
valuesOutput.style.overflowY = 'auto';
repeatOutput.appendChild(valuesOutput);

// シーケンスの繰り返し
of('A', 'B', 'C').pipe(
  // 繰り返し回数をインクリメント
  tap(() => {
    if (repeatCount === 0 || repeatCount === 3) {
      // 最初と最後の繰り返しでカウンタをリセット
      repeatCount = 1;
    } else {
      repeatCount++;
    }
    repeatCountDisplay.textContent = `繰り返し回数: ${repeatCount}`;
    
    // 繰り返しの区切り
    if (repeatCount === 1 && valuesOutput.childElementCount > 0) {
      const separator = document.createElement('hr');
      valuesOutput.appendChild(separator);
    }
  }),
  // 3回繰り返す
  repeat(3)
).subscribe(val => {
  const valueItem = document.createElement('div');
  valueItem.textContent = `値: ${val} (繰り返し ${repeatCount})`;
  valuesOutput.appendChild(valueItem);
});
```

`repeat`は、特定のシーケンスを繰り返す必要がある場合（ポーリング、アニメーション、定期的な処理など）に便利です。特に、固定回数の繰り返しが必要な場合に適しています。

## retry - エラー時の再試行

`retry`オペレーターは、エラーが発生した場合にソースObservableを指定した回数だけ再購読します。ネットワークリクエストなど、一時的な障害から回復する可能性がある操作に有用です。

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs/operators';

// 出力表示エリア
const retryOutput = document.createElement('div');
retryOutput.innerHTML = '<h3>retry の例 (APIリクエストシミュレーション):</h3>';
document.body.appendChild(retryOutput);

// リクエスト状態表示
const requestStatus = document.createElement('div');
requestStatus.style.marginTop = '10px';
requestStatus.style.padding = '10px';
requestStatus.style.border = '1px solid #ddd';
requestStatus.style.maxHeight = '200px';
requestStatus.style.overflowY = 'auto';
retryOutput.appendChild(requestStatus);

// ランダムに成功または失敗するAPIリクエストをシミュレート
let attemptCount = 0;

function simulateRequest() {
  attemptCount++;

  const logEntry = document.createElement('div');
  logEntry.textContent = `試行 #${attemptCount} リクエスト送信中...`;
  requestStatus.appendChild(logEntry);

  // 80%の確率で失敗する過酷な状況をシミュレート
  return interval(1000).pipe(
    mergeMap((_) => {
      // ランダムに成功または失敗
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        const errorMsg = document.createElement('div');
        errorMsg.textContent = `試行 #${attemptCount} 失敗: ネットワークエラー`;
        errorMsg.style.color = 'red';
        requestStatus.appendChild(errorMsg);

        return throwError(() => new Error('ネットワークエラー'));
      } else {
        const successMsg = document.createElement('div');
        successMsg.textContent = `試行 #${attemptCount} 成功！`;
        successMsg.style.color = 'green';
        requestStatus.appendChild(successMsg);

        return of({ id: 1, name: 'データが正常に取得されました' });
      }
    }),
    // 最大3回再試行
    retry(3),
    // 全再試行が失敗した場合
    catchError((err) => {
      const finalError = document.createElement('div');
      finalError.textContent = `すべての再試行に失敗しました: ${err.message}`;
      finalError.style.color = 'red';
      finalError.style.fontWeight = 'bold';
      requestStatus.appendChild(finalError);

      return of({ error: true, message: '再試行が失敗しました' });
    })
  );
}

// リクエスト開始ボタン
const startButton = document.createElement('button');
startButton.textContent = 'リクエスト開始';
startButton.style.padding = '8px 16px';
startButton.style.marginTop = '10px';
retryOutput.insertBefore(startButton, requestStatus);

// ボタンクリック時にリクエスト開始
startButton.addEventListener('click', () => {
  // リセット
  attemptCount = 0;
  requestStatus.innerHTML = '';
  startButton.disabled = true;

  simulateRequest().subscribe((result) => {
    const resultElement = document.createElement('div');
    if ('error' in result) {
      resultElement.textContent = `最終結果: ${result.message}`;
      resultElement.style.backgroundColor = '#ffebee';
    } else {
      resultElement.textContent = `最終結果: ${result.name}`;
      resultElement.style.backgroundColor = '#e8f5e9';
    }

    resultElement.style.padding = '10px';
    resultElement.style.marginTop = '10px';
    resultElement.style.borderRadius = '5px';
    requestStatus.appendChild(resultElement);

    // ボタンを再度有効化
    startButton.disabled = false;
  });
});

```

`retry`は、ネットワークリクエストやその他の一時的な障害が発生する可能性がある非同期操作で特に役立ちます。指定した回数だけ操作を再試行し、回復の機会を提供します。

## startWith - 初期値の提供

`startWith`オペレーターは、ソースObservableからの値を発行する前に、指定した値をまず最初に発行します。初期状態の設定や、ストリームが値を発行するまでのプレースホルダーとして便利です。

```ts
import { interval } from 'rxjs';
import { startWith, scan, take } from 'rxjs/operators';

// 出力表示エリア
const startWithOutput = document.createElement('div');
startWithOutput.innerHTML = '<h3>startWith の例:</h3>';
document.body.appendChild(startWithOutput);

// カウンター表示エリア
const counterDisplay = document.createElement('div');
counterDisplay.style.fontSize = '24px';
counterDisplay.style.fontWeight = 'bold';
counterDisplay.style.textAlign = 'center';
counterDisplay.style.padding = '20px';
counterDisplay.style.border = '1px solid #ddd';
counterDisplay.style.borderRadius = '5px';
counterDisplay.style.margin = '10px 0';
startWithOutput.appendChild(counterDisplay);

// 値のリスト表示エリア
const valuesList = document.createElement('div');
valuesList.style.marginTop = '10px';
startWithOutput.appendChild(valuesList);

// カウンターストリーム (1秒ごと)
interval(1000).pipe(
  // 最初に100から開始
  startWith(-1),
  // 各値を前の値に1を加える
  scan((acc, curr) => acc + 1, 100),
  // 10回で終了
  take(10)
).subscribe(count => {
  // カウンター表示を更新
  counterDisplay.textContent = count.toString();
  
  // 値をリストに追加
  const valueItem = document.createElement('div');
  
  if (count === 100) {
    valueItem.textContent = `初期値: ${count} (startWithで追加)`;
    valueItem.style.color = 'blue';
  } else {
    valueItem.textContent = `次の値: ${count}`;
  }
  
  valuesList.appendChild(valueItem);
});
```

`startWith`は、初期状態の設定、ローディング状態の表示、または既存のデータと新しいデータの結合などに使用できます。特に状態管理やUIコンポーネントの初期化に便利です。

## toArray - 値の配列への変換

`toArray`オペレーターは、ソースObservableから発行されたすべての値を集め、ソースが完了したときに1つの配列として発行します。

```ts
import { interval } from 'rxjs';
import { take, toArray } from 'rxjs/operators';

// 出力表示エリア
const toArrayOutput = document.createElement('div');
toArrayOutput.innerHTML = '<h3>toArray の例:</h3>';
document.body.appendChild(toArrayOutput);

// 個別の値表示エリア
const individualValues = document.createElement('div');
individualValues.innerHTML = '<h4>個別の値:</h4>';
toArrayOutput.appendChild(individualValues);

// 配列結果表示エリア
const arrayResult = document.createElement('div');
arrayResult.innerHTML = '<h4>配列結果:</h4>';
arrayResult.style.marginTop = '20px';
toArrayOutput.appendChild(arrayResult);

// 個別の値を購読
interval(500).pipe(
  take(5)
).subscribe(val => {
  const valueItem = document.createElement('div');
  valueItem.textContent = `値: ${val}`;
  individualValues.appendChild(valueItem);
});

// 同じストリームを配列として購読
interval(500).pipe(
  take(5),
  toArray()
).subscribe(array => {
  const resultItem = document.createElement('div');
  resultItem.textContent = `結果配列: [${array.join(', ')}]`;
  resultItem.style.fontWeight = 'bold';
  resultItem.style.padding = '10px';
  resultItem.style.backgroundColor = '#f5f5f5';
  resultItem.style.borderRadius = '5px';
  arrayResult.appendChild(resultItem);
  
  // 配列の要素を個別に表示
  const arrayItems = document.createElement('div');
  arrayItems.style.marginTop = '10px';
  
  array.forEach((item, index) => {
    const arrayItem = document.createElement('div');
    arrayItem.textContent = `array[${index}] = ${item}`;
    arrayItems.appendChild(arrayItem);
  });
  
  arrayResult.appendChild(arrayItems);
});
```

`toArray`は、すべての値を収集して一度に処理したい場合や、一連の非同期操作の結果をバッチ処理したい場合に便利です。特にHTTPレスポンスの処理やフォーム送信などで役立ちます。

## 実用的なユースケース

### ローディング状態の管理

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

### フォーム検証と送信

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

ユーティリティオペレーターは、RxJSでのプログラミングをより効率的かつ堅牢にするための重要なツールです。これらのオペレーターを適切に組み合わせることで、以下のような利点が得られます：

1. **デバッグの容易さ**: `tap`を使用することで、ストリームの中間状態を簡単に確認できます。
2. **エラー耐性**: `retry`、`timeout`、`catchError`を組み合わせることで、堅牢なエラー処理が可能になります。
3. **リソース管理**: `finalize`を使用することで、リソースの適切なクリーンアップが保証されます。
4. **UIの応答性向上**: `startWith`、`delay`などを使用して、ユーザー体験を向上させることができます。
5. **コードの可読性向上**: ユーティリティオペレーターを使用することで、副作用と純粋なデータ変換を明確に分離できます。

これらのオペレーターは、単独で使用するよりも、他のオペレーターと組み合わせて使用することで、その真価を発揮します。実際のアプリケーション開発では、複数のオペレーターを組み合わせて、複雑な非同期処理フローを管理することが一般的です。