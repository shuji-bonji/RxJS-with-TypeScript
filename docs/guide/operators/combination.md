# 結合オペレーター

結合オペレーターは、複数のObservableのストリームを組み合わせて一つの新しいストリームを生成するためのRxJSのオペレーターです。これらのオペレーターを使うことで、異なるデータソースを効果的に連携させることができます。

## 基本的な結合オペレーター

### merge

`merge`オペレーターは、複数のObservableからの値を一つのObservableにマージします。すべてのソースからの値が時間順に出力されます。

```ts
import { merge, interval } from 'rxjs';
import { map, take } from 'rxjs/operators';

// 出力表示エリアを作成
const output = document.createElement('div');
document.body.appendChild(output);

// 2つの異なるストリーム
const source1$ = interval(1000).pipe(
  map(val => `ストリーム1: ${val}`),
  take(5)
);

const source2$ = interval(1500).pipe(
  map(val => `ストリーム2: ${val}`),
  take(5)
);

// ストリームをマージ
merge(source1$, source2$).subscribe(value => {
  const item = document.createElement('div');
  item.textContent = value;
  output.appendChild(item);
});

// 出力:
// ストリーム1: 0
// ストリーム2: 0
// ストリーム1: 1
// ストリーム1: 2
// ストリーム2: 1
// ストリーム1: 3
// ストリーム2: 2
// ストリーム1: 4
// ストリーム2: 3
// ストリーム2: 4
```

### concat

`concat`オペレーターは、指定された順序で複数のObservableを連結します。前のObservableが完了した後に次のObservableからの値の出力が始まります。

```ts
import { concat, of } from 'rxjs';
import { delay } from 'rxjs/operators';

// 出力表示エリアを作成
const concatOutput = document.createElement('div');
concatOutput.innerHTML = '<h3>concat の例:</h3>';
document.body.appendChild(concatOutput);

// 3つのObservable
const source1$ = of('A1', 'A2', 'A3').pipe(delay(500));
const source2$ = of('B1', 'B2');
const source3$ = of('C1', 'C2', 'C3', 'C4');

// 順番に連結
concat(source1$, source2$, source3$).subscribe(value => {
  const item = document.createElement('div');
  item.textContent = value;
  concatOutput.appendChild(item);
});

// 出力:
// A1
// A2
// A3
// B1
// B2
// C1
// C2
// C3
// C4
```

### combineLatest

`combineLatest`オペレーターは、各ソースObservableの最新の値をまとめて配列として出力します。いずれかのソースから新しい値が発行されるたびに、すべてのソースの最新値の組み合わせが出力されます。

```ts
import { combineLatest, timer, interval } from 'rxjs';
import { map, take } from 'rxjs/operators';

// 出力表示エリアを作成
const combineOutput = document.createElement('div');
combineOutput.innerHTML = '<h3>combineLatest の例:</h3>';
document.body.appendChild(combineOutput);

// 2つの異なるタイミングのストリーム
const source1$ = timer(0, 2000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = timer(500, 1000).pipe(
  map(val => `B${val}`),
  take(5)
);

// 最新の値を組み合わせる
combineLatest([source1$, source2$]).subscribe(([value1, value2]) => {
  const item = document.createElement('div');
  item.textContent = `組み合わせ: ${value1}, ${value2}`;
  combineOutput.appendChild(item);
});

// 出力:
// 組み合わせ: A0, B0
// 組み合わせ: A0, B1
// 組み合わせ: A0, B2
// 組み合わせ: A1, B2
// 組み合わせ: A1, B3
// 組み合わせ: A1, B4
// 組み合わせ: A2, B4
```

### zip

`zip`オペレーターは、複数のObservableの対応する値を配列にまとめます。各ソースから値が到着するまで待機し、対応するインデックスの値をまとめて出力します。

```ts
import { zip, of, interval } from 'rxjs';
import { take, map } from 'rxjs/operators';

// 出力表示エリアを作成
const zipOutput = document.createElement('div');
zipOutput.innerHTML = '<h3>zip の例:</h3>';
document.body.appendChild(zipOutput);

// 2つのストリーム
const source1$ = of('A', 'B', 'C', 'D');
const source2$ = interval(1000).pipe(
  map(val => val * 10),
  take(4)
);

// インデックスごとに組み合わせる
zip(source1$, source2$).subscribe(([letter, number]) => {
  const item = document.createElement('div');
  item.textContent = `${letter} - ${number}`;
  zipOutput.appendChild(item);
});

// 出力:
// A - 0
// B - 10
// C - 20
// D - 30
```

## 高度な結合オペレーター

### withLatestFrom

`withLatestFrom`オペレーターは、メインストリームの値が発行されるたびに、別のストリームの最新の値と組み合わせて出力します。メインストリームの値をトリガーとし、他のストリームからの値はその時点の最新値が使われます。

```ts
import { interval, fromEvent } from 'rxjs';
import { withLatestFrom, map, take } from 'rxjs/operators';

// UI要素を作成
const wlfContainer = document.createElement('div');
wlfContainer.innerHTML = '<h3>withLatestFrom の例:</h3>';
document.body.appendChild(wlfContainer);

const clickButton = document.createElement('button');
clickButton.textContent = 'クリックしてください';
clickButton.style.padding = '8px 16px';
wlfContainer.appendChild(clickButton);

const resultContainer = document.createElement('div');
resultContainer.style.marginTop = '10px';
wlfContainer.appendChild(resultContainer);

// カウンターストリーム（1秒ごとに増加）
const counter$ = interval(1000).pipe(take(10));

// クリックイベントストリーム
const click$ = fromEvent(clickButton, 'click');

// クリック時にカウンターの現在値を取得
click$.pipe(
  withLatestFrom(counter$),
  map(([event, count]) => `クリック時のカウンター値: ${count}`)
).subscribe(message => {
  const item = document.createElement('div');
  item.textContent = message;
  resultContainer.appendChild(item);
});
```

### forkJoin

`forkJoin`オペレーターは、すべてのソースObservableが完了するのを待ち、それぞれの最後の値を配列として出力します。すべてのAPIリクエストが完了するのを待つような場合に便利です。

```ts
import { forkJoin, of, timer } from 'rxjs';
import { delay, take } from 'rxjs/operators';

// 出力表示エリアを作成
const forkOutput = document.createElement('div');
forkOutput.innerHTML = '<h3>forkJoin の例:</h3>';
document.body.appendChild(forkOutput);

// 3つの異なるAPIレスポンスをシミュレート
const user$ = of({ id: 1, name: '山田太郎' }).pipe(delay(2000)); // 2秒
const posts$ = of([
  { id: 1, title: '投稿1' },
  { id: 2, title: '投稿2' }
]).pipe(delay(1500)); // 1.5秒
const weather$ = of({ temp: 22, condition: '晴れ' }).pipe(delay(1000)); // 1秒

// ローディングメッセージ
const loading = document.createElement('div');
loading.textContent = 'データを読み込み中...';
loading.style.color = 'blue';
forkOutput.appendChild(loading);

// すべてのAPIリクエストが完了するまで待機
forkJoin({
  user: user$,
  posts: posts$,
  weather: weather$
}).subscribe(result => {
  // ローディングメッセージを削除
  forkOutput.removeChild(loading);
  
  const output = document.createElement('pre');
  output.textContent = JSON.stringify(result, null, 2);
  output.style.background = '#f5f5f5';
  output.style.padding = '10px';
  output.style.borderRadius = '5px';
  forkOutput.appendChild(output);
  
  const message = document.createElement('div');
  message.textContent = `ユーザー: ${result.user.name}、天気: ${result.weather.condition}、投稿数: ${result.posts.length}`;
  forkOutput.appendChild(message);
});

// 出力:
// {
//   "user": {
//     "id": 1,
//     "name": "山田太郎"
//   },
//   "posts": [
//     {
//       "id": 1,
//       "title": "投稿1"
//     },
//     {
//       "id": 2,
//       "title": "投稿2"
//     }
//   ],
//   "weather": {
//     "temp": 22,
//     "condition": "晴れ"
//   }
// }
// ユーザー: 山田太郎、天気: 晴れ、投稿数: 2
```

### race

`race`オペレーターは、複数のObservableの中で最初に値を発行したObservableからのみ値を出力し、他のObservableは無視します。

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs/operators';

// 出力表示エリアを作成
const raceOutput = document.createElement('div');
raceOutput.innerHTML = '<h3>race の例:</h3>';
document.body.appendChild(raceOutput);

// 異なるタイミングで始まる3つのストリーム
const slow$ = timer(5000).pipe(map(() => 'ゆっくり (5秒)'));
const medium$ = timer(3000).pipe(map(() => '普通 (3秒)'));
const fast$ = timer(2000).pipe(map(() => '速い (2秒)'));

const startTime = Date.now();

// メッセージを表示
const waiting = document.createElement('div');
waiting.textContent = 'レース開始...';
raceOutput.appendChild(waiting);

// 一番早く発行したストリームが勝つ
race(slow$, medium$, fast$).subscribe(winner => {
  const endTime = Date.now();
  const elapsed = (endTime - startTime) / 1000;
  
  const result = document.createElement('div');
  result.textContent = `勝者: ${winner} (経過時間: ${elapsed.toFixed(2)}秒)`;
  result.style.fontWeight = 'bold';
  result.style.color = 'green';
  raceOutput.appendChild(result);
  
  const explanation = document.createElement('div');
  explanation.textContent = '最初に値を発行したObservableのみが採用され、他は無視されます。';
  raceOutput.appendChild(explanation);
});

// 出力:
// 勝者: 速い (2秒) (経過時間: 2.00秒)
// 最初に値を発行したObservableのみが採用され、他は無視されます。
```

## 実践的なユースケース

### フォーム入力の検証とAPIリクエスト

`combineLatest`を使用して複数のフォーム入力を検証する例です。

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, debounceTime, startWith } from 'rxjs/operators';

// フォームUI作成
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>ユーザー登録フォーム:</h3>';
document.body.appendChild(formContainer);

// 名前入力
const nameLabel = document.createElement('label');
nameLabel.textContent = '名前: ';
formContainer.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.id = 'name';
nameInput.style.marginBottom = '10px';
nameInput.style.marginLeft = '5px';
formContainer.appendChild(nameInput);
formContainer.appendChild(document.createElement('br'));

// メール入力
const emailLabel = document.createElement('label');
emailLabel.textContent = 'メール: ';
formContainer.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.id = 'email';
emailInput.style.marginBottom = '10px';
emailInput.style.marginLeft = '5px';
formContainer.appendChild(emailInput);
formContainer.appendChild(document.createElement('br'));

// パスワード入力
const passwordLabel = document.createElement('label');
passwordLabel.textContent = 'パスワード: ';
formContainer.appendChild(passwordLabel);

const passwordInput = document.createElement('input');
passwordInput.type = 'password';
passwordInput.id = 'password';
passwordInput.style.marginLeft = '5px';
formContainer.appendChild(passwordInput);
formContainer.appendChild(document.createElement('br'));

// 送信ボタン
const submitButton = document.createElement('button');
submitButton.textContent = '登録';
submitButton.disabled = true;
submitButton.style.marginTop = '15px';
submitButton.style.padding = '8px 16px';
formContainer.appendChild(submitButton);

// バリデーションメッセージ
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// 名前の検証
const name$ = fromEvent(nameInput, 'input').pipe(
  map(event => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: value.length >= 2,
      error: value.length < 2 ? '名前は2文字以上入力してください' : null
    };
  }),
  startWith({ value: '', valid: false, error: '名前は2文字以上入力してください' })
);

// メールの検証
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map(event => {
    const value = (event.target as HTMLInputElement).value.trim();
    return {
      value,
      valid: emailRegex.test(value),
      error: !emailRegex.test(value) ? '有効なメールアドレスを入力してください' : null
    };
  }),
  startWith({ value: '', valid: false, error: '有効なメールアドレスを入力してください' })
);

// パスワードの検証
const password$ = fromEvent(passwordInput, 'input').pipe(
  map(event => {
    const value = (event.target as HTMLInputElement).value;
    return {
      value,
      valid: value.length >= 6,
      error: value.length < 6 ? 'パスワードは6文字以上入力してください' : null
    };
  }),
  startWith({ value: '', valid: false, error: 'パスワードは6文字以上入力してください' })
);

// 全フィールドの検証状態を組み合わせる
combineLatest([name$, email$, password$]).pipe(
  debounceTime(300)
).subscribe(([nameState, emailState, passwordState]) => {
  // フォームが有効かどうか
  const isFormValid = nameState.valid && emailState.valid && passwordState.valid;
  submitButton.disabled = !isFormValid;
  
  // エラーメッセージを表示
  if (!isFormValid) {
    const errors = [
      nameState.error,
      emailState.error,
      passwordState.error
    ].filter(error => error !== null);
    
    validationMessage.textContent = errors.join('\n');
  } else {
    validationMessage.textContent = '';
  }
});

// 送信ボタンのクリックイベント
fromEvent(submitButton, 'click').subscribe(() => {
  const formData = {
    name: nameInput.value,
    email: emailInput.value,
    password: passwordInput.value
  };
  
  // フォームデータを表示（実際はAPIに送信）
  const successMessage = document.createElement('div');
  successMessage.textContent = '登録が完了しました！';
  successMessage.style.color = 'green';
  successMessage.style.fontWeight = 'bold';
  successMessage.style.marginTop = '10px';
  formContainer.appendChild(successMessage);
  
  console.log('送信データ:', formData);
});
```

### 同時リクエストとローディング状態の管理

`forkJoin`を使用して複数のAPIリクエストを並行して処理し、結果をまとめる例です。

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, delay, finalize } from 'rxjs/operators';

// UI要素の作成
const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>複数APIリクエストの例:</h3>';
document.body.appendChild(apiContainer);

const loadButton = document.createElement('button');
loadButton.textContent = 'データ読み込み';
loadButton.style.padding = '8px 16px';
apiContainer.appendChild(loadButton);

const loadingIndicator = document.createElement('div');
loadingIndicator.style.margin = '10px 0';
loadingIndicator.style.display = 'none';
apiContainer.appendChild(loadingIndicator);

const resultContainer = document.createElement('div');
apiContainer.appendChild(resultContainer);

// APIリクエストをシミュレート
function fetchUser(id) {
  // 成功するリクエスト
  return of({
    id,
    name: `ユーザー${id}`,
    email: `user${id}@example.com`
  }).pipe(
    delay(2000) // 2秒の遅延
  );
}

function fetchPosts(userId) {
  // 成功するリクエスト
  return of([
    { id: 1, title: `${userId}の投稿1`, content: '内容...' },
    { id: 2, title: `${userId}の投稿2`, content: '内容...' }
  ]).pipe(
    delay(1500) // 1.5秒の遅延
  );
}

function fetchWeather(city) {
  // 時々失敗するリクエスト
  const shouldFail = Math.random() > 0.7;
  
  if (shouldFail) {
    return throwError(() => new Error('天気データの取得に失敗しました')).pipe(
      delay(1000)
    );
  }
  
  return of({
    city,
    temp: Math.round(15 + Math.random() * 10),
    condition: ['晴れ', '曇り', '雨'][Math.floor(Math.random() * 3)]
  }).pipe(
    delay(1000) // 1秒の遅延
  );
}

// ボタンクリック時に複数リクエストを実行
loadButton.addEventListener('click', () => {
  // UIをリセット
  resultContainer.innerHTML = '';
  loadingIndicator.style.display = 'block';
  loadingIndicator.textContent = 'データ読み込み中...';
  loadButton.disabled = true;
  
  // 複数のAPIリクエストを同時に実行
  forkJoin({
    user: fetchUser(1),
    posts: fetchPosts(1),
    weather: fetchWeather('東京').pipe(
      // エラー処理
      catchError(error => {
        console.error('天気APIエラー:', error);
        return of({ error: error.message });
      })
    )
  }).pipe(
    // 完了時の処理
    finalize(() => {
      loadingIndicator.style.display = 'none';
      loadButton.disabled = false;
    })
  ).subscribe(results => {
    // ユーザー情報の表示
    const userInfo = document.createElement('div');
    userInfo.innerHTML = `
      <h4>ユーザー情報</h4>
      <p>名前: ${results.user.name}</p>
      <p>メール: ${results.user.email}</p>
    `;
    userInfo.style.margin = '10px 0';
    userInfo.style.padding = '10px';
    userInfo.style.backgroundColor = '#f0f0f0';
    userInfo.style.borderRadius = '5px';
    resultContainer.appendChild(userInfo);
    
    // 投稿の表示
    const postsInfo = document.createElement('div');
    postsInfo.innerHTML = `
      <h4>投稿一覧 (${results.posts.length}件)</h4>
      <ul>
        ${results.posts.map(post => `<li>${post.title}</li>`).join('')}
      </ul>
    `;
    postsInfo.style.margin = '10px 0';
    postsInfo.style.padding = '10px';
    postsInfo.style.backgroundColor = '#f0f0f0';
    postsInfo.style.borderRadius = '5px';
    resultContainer.appendChild(postsInfo);
    
    // 天気情報の表示
    const weatherInfo = document.createElement('div');
    
    if (results.weather.error) {
      weatherInfo.innerHTML = `
        <h4>天気情報</h4>
        <p style="color: red;">エラー: ${results.weather.error}</p>
      `;
    } else {
      weatherInfo.innerHTML = `
        <h4>天気情報</h4>
        <p>都市: ${results.weather.city}</p>
        <p>気温: ${results.weather.temp}°C</p>
        <p>状態: ${results.weather.condition}</p>
      `;
    }
    
    weatherInfo.style.margin = '10px 0';
    weatherInfo.style.padding = '10px';
    weatherInfo.style.backgroundColor = '#f0f0f0';
    weatherInfo.style.borderRadius = '5px';
    resultContainer.appendChild(weatherInfo);
  });
});
```

### キャンセル可能な検索機能

`withLatestFrom`と`race`を組み合わせて、タイムアウトやキャンセル可能な検索機能を実装する例です。

```ts
import { fromEvent, timer, race, of, EMPTY } from 'rxjs';
import { map, debounceTime, switchMap, tap, delay, catchError, takeUntil } from 'rxjs/operators';

// 検索UI作成
const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>キャンセル可能な検索:</h3>';
document.body.appendChild(searchContainer);

// 検索入力欄
const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = '検索語を入力...';
searchInput.style.padding = '8px';
searchInput.style.width = '250px';
searchContainer.appendChild(searchInput);

// キャンセルボタン
const cancelButton = document.createElement('button');
cancelButton.textContent = 'キャンセル';
cancelButton.style.marginLeft = '10px';
cancelButton.style.padding = '8px 16px';
searchContainer.appendChild(cancelButton);

// 検索結果エリア
const resultsContainer = document.createElement('div');
resultsContainer.style.marginTop = '10px';
resultsContainer.style.minHeight = '200px';
resultsContainer.style.padding = '10px';
resultsContainer.style.border = '1px solid #ddd';
resultsContainer.style.borderRadius = '5px';
searchContainer.appendChild(resultsContainer);

// 検索リクエストをシミュレート
function searchApi(term) {
  console.log(`「${term}」の検索を開始...`);
  
  // 検索結果をシミュレート
  return of([
    `「${term}」の検索結果1`,
    `「${term}」の検索結果2`,
    `「${term}」の検索結果3`
  ]).pipe(
    // 2〜5秒の遅延をランダムで設定
    delay(2000 + Math.random() * 3000),
    // エラー対応
    catchError(err => {
      console.error('検索エラー:', err);
      return EMPTY;
    })
  );
}

// キャンセルイベント
const cancel$ = fromEvent(cancelButton, 'click');

// 検索イベント
const search$ = fromEvent(searchInput, 'input').pipe(
  // 入力値を取得
  map(event => (event.target as HTMLInputElement).value.trim()),
  // 300ms待機
  debounceTime(300),
  // 空の検索は無視
  tap(term => {
    if (term === '') {
      resultsContainer.innerHTML = '<p>検索語を入力してください</p>';
    }
  }),
  // 空の検索は処理しない
  switchMap(term => {
    if (term === '') {
      return EMPTY;
    }
    
    // 検索中表示
    resultsContainer.innerHTML = '<p>検索中...</p>';
    
    // タイムアウト処理（5秒）
    const timeout$ = timer(5000).pipe(
      tap(() => console.log('検索がタイムアウトしました')),
      map(() => ({ type: 'timeout', results: null }))
    );
    
    // APIリクエスト
    const request$ = searchApi(term).pipe(
      map(results => ({ type: 'success', results })),
      // キャンセルボタンが押されたら中止
      takeUntil(cancel$.pipe(
        tap(() => {
          console.log('検索がキャンセルされました');
          resultsContainer.innerHTML = '<p>検索はキャンセルされました</p>';
        })
      ))
    );
    
    // タイムアウトかリクエスト完了のどちらか早い方
    return race(request$, timeout$);
  })
).subscribe(response => {
  if (response.type === 'success') {
    // 検索成功
    resultsContainer.innerHTML = '<h4>検索結果:</h4>';
    
    if (response.results.length === 0) {
      resultsContainer.innerHTML += '<p>結果が見つかりませんでした</p>';
    } else {
      const list = document.createElement('ul');
      response.results.forEach(result => {
        const item = document.createElement('li');
        item.textContent = result;
        list.appendChild(item);
      });
      resultsContainer.appendChild(list);
    }
  } else if (response.type === 'timeout') {
    // タイムアウト
    resultsContainer.innerHTML = '<p style="color: red;">検索がタイムアウトしました。もう一度お試しください。</p>';
  }
});
```

## 結合オペレーターの比較と選択ガイド

| オペレーター | タイミング | 出力 | ユースケース |
|------------|------------|-----|------------|
| `merge` | 同時実行 | 発生順に出力 | 複数ソースイベントの同時監視 |
| `concat` | 順次実行 | 順番通りに出力 | 順序が重要な非同期タスク |
| `combineLatest` | すべてのソースから少なくとも1つの値が必要 | すべての最新値の組み合わせ | フォーム入力の検証 |
| `zip` | すべてのソースから対応するインデックスの値が必要 | インデックスごとの値の組み合わせ | 関連データの同期 |
| `withLatestFrom` | メインソースの値発行時 | メインの値と他ソースの最新値 | 補助データの組み合わせ |
| `forkJoin` | すべてのソースが完了時 | 各ソースの最後の値 | 複数APIリクエスト |
| `race` | 最初に値を発行したソースのみ | 勝者のストリームの値のみ | タイムアウト、キャンセル処理 |

### オペレーター選択の判断フロー

1. **すべてのソースから同時に値を受け取りたい？**
   - Yes → `merge`
   - No → 次へ

2. **ソースの順序を保持したい？**
   - Yes → `concat`
   - No → 次へ

3. **各ソースの最新値の組み合わせが必要？**
   - Yes → いつ組み合わせる？
     - 任意のソースの新しい値ごと → `combineLatest`
     - 特定のメインストリーム値ごと → `withLatestFrom`
   - No → 次へ

4. **インデックス順で対応する値が必要？**
   - Yes → `zip`
   - No → 次へ

5. **すべてのソースが完了したあとの結果が必要？**
   - Yes → `forkJoin`
   - No → 次へ

6. **複数の代替ソースから最速のものだけ必要？**
   - Yes → `race`
   - No → 目的を再検討

## 高度な結合パターン

### スイッチング戦略

複数のデータソースを動的に切り替える例です。

```ts
import { fromEvent, merge, interval, of } from 'rxjs';
import { map, switchMap, take, tap } from 'rxjs/operators';

// UI要素の作成
const switchingContainer = document.createElement('div');
switchingContainer.innerHTML = '<h3>データソース切り替え:</h3>';
document.body.appendChild(switchingContainer);

// ボタン作成
const source1Button = document.createElement('button');
source1Button.textContent = 'ソース 1';
source1Button.style.margin = '5px';
source1Button.style.padding = '5px 10px';
switchingContainer.appendChild(source1Button);

const source2Button = document.createElement('button');
source2Button.textContent = 'ソース 2';
source2Button.style.margin = '5px';
source2Button.style.padding = '5px 10px';
switchingContainer.appendChild(source2Button);

const source3Button = document.createElement('button');
source3Button.textContent = 'ソース 3';
source3Button.style.margin = '5px';
source3Button.style.padding = '5px 10px';
switchingContainer.appendChild(source3Button);

// 結果表示エリア
const resultsArea = document.createElement('div');
resultsArea.style.marginTop = '10px';
resultsArea.style.minHeight = '150px';
resultsArea.style.padding = '10px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.backgroundColor = '#f9f9f9';
switchingContainer.appendChild(resultsArea);

// 3つのデータソース
function createSource1() {
  return interval(1000).pipe(
    take(5),
    map(val => `ソース1: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '#c8e6c9';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource2() {
  return interval(500).pipe(
    take(8),
    map(val => `ソース2: ${val}`),
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '#bbdefb';
      source3Button.style.backgroundColor = '';
    })
  );
}

function createSource3() {
  return of('ソース3: A', 'ソース3: B', 'ソース3: C').pipe(
    tap(() => {
      source1Button.style.backgroundColor = '';
      source2Button.style.backgroundColor = '';
      source3Button.style.backgroundColor = '#ffccbc';
    })
  );
}

// ボタンクリックイベント
const source1Click$ = fromEvent(source1Button, 'click').pipe(
  map(() => 1)
);

const source2Click$ = fromEvent(source2Button, 'click').pipe(
  map(() => 2)
);

const source3Click$ = fromEvent(source3Button, 'click').pipe(
  map(() => 3)
);

// ボタンクリックをマージ
merge(
  source1Click$,
  source2Click$,
  source3Click$
).pipe(
  // 選択されたソースに切り替え
  switchMap(sourceId => {
    // 結果エリアをクリア
    resultsArea.innerHTML = '';
    
    // 選択されたソースを返す
    switch (sourceId) {
      case 1:
        return createSource1();
      case 2:
        return createSource2();
      case 3:
        return createSource3();
      default:
        return of('ソースが選択されていません');
    }
  })
).subscribe(value => {
  // 結果を表示
  const item = document.createElement('div');
  item.textContent = value;
  item.style.padding = '5px';
  item.style.margin = '2px 0';
  item.style.backgroundColor = 'white';
  item.style.borderRadius = '3px';
  resultsArea.appendChild(item);
});

// 初期メッセージ
const initialMessage = document.createElement('div');
initialMessage.textContent = 'ボタンをクリックしてデータソースを選択してください';
initialMessage.style.color = '#666';
resultsArea.appendChild(initialMessage);
```

### 条件付きマージ

`merge`と`filter`を組み合わせて、条件に基づいたデータソースの選択をする例です。

```ts
import { merge, interval, fromEvent, from } from 'rxjs';
import { map, filter, takeUntil, withLatestFrom, startWith } from 'rxjs/operators';

// UI要素の作成
const conditionalContainer = document.createElement('div');
conditionalContainer.innerHTML = '<h3>条件付きマージ:</h3>';
document.body.appendChild(conditionalContainer);

// フィルター設定
const filterDiv = document.createElement('div');
filterDiv.style.marginBottom = '10px';
conditionalContainer.appendChild(filterDiv);

// チェックボックス作成
const slowCheck = document.createElement('input');
slowCheck.type = 'checkbox';
slowCheck.id = 'slowCheck';
slowCheck.checked = true;
filterDiv.appendChild(slowCheck);

const slowLabel = document.createElement('label');
slowLabel.htmlFor = 'slowCheck';
slowLabel.textContent = '遅いソース';
slowLabel.style.marginRight = '15px';
filterDiv.appendChild(slowLabel);

const fastCheck = document.createElement('input');
fastCheck.type = 'checkbox';
fastCheck.id = 'fastCheck';
fastCheck.checked = true;
filterDiv.appendChild(fastCheck);

const fastLabel = document.createElement('label');
fastLabel.htmlFor = 'fastCheck';
fastLabel.textContent = '速いソース';
fastLabel.style.marginRight = '15px';
filterDiv.appendChild(fastLabel);

const clickCheck = document.createElement('input');
clickCheck.type = 'checkbox';
clickCheck.id = 'clickCheck';
clickCheck.checked = true;
filterDiv.appendChild(clickCheck);

const clickLabel = document.createElement('label');
clickLabel.htmlFor = 'clickCheck';
clickLabel.textContent = 'クリックイベント';
filterDiv.appendChild(clickLabel);

// 停止ボタン
const stopButton = document.createElement('button');
stopButton.textContent = '停止';
stopButton.style.marginLeft = '15px';
filterDiv.appendChild(stopButton);

// 結果表示エリア
const conditionalResults = document.createElement('div');
conditionalResults.style.height = '200px';
conditionalResults.style.overflowY = 'auto';
conditionalResults.style.padding = '10px';
conditionalResults.style.border = '1px solid #ddd';
conditionalResults.style.backgroundColor = '#f9f9f9';
conditionalContainer.appendChild(conditionalResults);

// 3つのデータソース
// 1. 遅いソース (1秒ごと)
const slow$ = interval(1000).pipe(
  map(val => ({ type: 'slow', value: val }))
);

// 2. 速いソース (300ミリ秒ごと)
const fast$ = interval(300).pipe(
  map(val => ({ type: 'fast', value: val }))
);

// 3. クリックイベント
const click$ = fromEvent(document.body, 'click').pipe(
  map(event => ({ 
    type: 'click', 
    value: { 
      x: (event as MouseEvent).clientX, 
      y: (event as MouseEvent).clientY 
    } 
  }))
);

// チェックボックスの状態を監視
const slowEnabled$ = fromEvent(slowCheck, 'change').pipe(
  map(event => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const fastEnabled$ = fromEvent(fastCheck, 'change').pipe(
  map(event => (event.target as HTMLInputElement).checked),
  startWith(true)
);

const clickEnabled$ = fromEvent(clickCheck, 'change').pipe(
  map(event => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// 停止イベント
const stop$ = fromEvent(stopButton, 'click');

// 条件付きマージ
merge(
  // 遅いソースと有効状態を組み合わせ
  slow$.pipe(
    withLatestFrom(slowEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),
  
  // 速いソースと有効状態を組み合わせ
  fast$.pipe(
    withLatestFrom(fastEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  ),
  
  // クリックソースと有効状態を組み合わせ
  click$.pipe(
    withLatestFrom(clickEnabled$),
    filter(([_, enabled]) => enabled),
    map(([event]) => event)
  )
).pipe(
  takeUntil(stop$)
).subscribe(event => {
  // 結果を表示
  const item = document.createElement('div');
  
  switch (event.type) {
    case 'slow':
      item.textContent = `遅いソース: ${event.value}`;
      item.style.color = '#1b5e20';
      break;
    case 'fast':
      item.textContent = `速いソース: ${event.value}`;
      item.style.color = '#0d47a1';
      break;
    case 'click':
      item.textContent = `クリック: X=${event.value.x}, Y=${event.value.y}`;
      item.style.color = '#bf360c';
      break;
  }
  
  item.style.padding = '3px';
  item.style.margin = '2px 0';
  conditionalResults.prepend(item); // 新しいものを上に表示
});
```

## まとめ

結合オペレーターは、複数のデータソースを組み合わせて一つのストリームにまとめるための強力なツールです。適切なオペレーターを選択することで、複雑な非同期データフローを簡潔かつ宣言的に表現できます。

結合オペレーターを使いこなすためのポイント：

1. **ユースケースに合わせた選択**: それぞれのオペレーターは特定のユースケースに最適化されています。目的に応じて適切なオペレーターを選びましょう。

2. **発行タイミングの理解**: 結合オペレーターの挙動は、値がいつ発行されるかに大きく依存します。各オペレーターの発行タイミングを理解することが重要です。

3. **エラー処理の考慮**: 結合されたストリームの一部でエラーが発生した場合の挙動（全体が失敗するか部分的に処理を続けるか）を検討しましょう。

4. **完了条件の把握**: 結合されたストリームがいつ完了するかを理解し、必要に応じて`takeUntil`などを使って明示的に完了させることも重要です。

5. **型安全性の活用**: TypeScriptを使用することで、結合オペレーターを型安全に扱うことができます。特に複雑な結合では型の恩恵が大きくなります。

結合オペレーターは、UIイベント処理、複数APIリクエスト、フォーム検証など、多くの実践的なシナリオで活用できます。これらのオペレーターをマスターすることで、RxJSのリアクティブプログラミングの真の力を引き出すことができるでしょう。