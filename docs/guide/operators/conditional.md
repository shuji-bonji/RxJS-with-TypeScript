# 条件オペレーター

RxJSの条件オペレーターは、ストリームの流れを制御し、特定の条件に基づいて異なる処理を行うためのツールです。これらのオペレーターを使用することで、複雑なビジネスロジックを宣言的に表現し、効率的なリアクティブアプリケーションを構築できます。

## iif - 条件に基づくObservableの選択

`iif`オペレーターは、条件式の評価結果に基づいて、2つのObservableのうちどちらかを選択します。JavaScriptの三項演算子（condition ? trueResult : falseResult）に似た機能を持ちます。

```ts
import { iif, of, EMPTY } from 'rxjs';

// 条件に基づいて異なるObservableを返す
function getDataBasedOnCondition(condition: boolean) {
  return iif(
    () => condition,
    of('条件はtrueです'),
    of('条件はfalseです')
  );
}

// UI要素を作成
const iifContainer = document.createElement('div');
iifContainer.innerHTML = '<h3>iif オペレーターの例:</h3>';
document.body.appendChild(iifContainer);

// true条件での実行
const trueButton = document.createElement('button');
trueButton.textContent = 'True条件で実行';
trueButton.style.marginRight = '10px';
iifContainer.appendChild(trueButton);

// false条件での実行
const falseButton = document.createElement('button');
falseButton.textContent = 'False条件で実行';
iifContainer.appendChild(falseButton);

// 結果表示エリア
const iifResult = document.createElement('div');
iifResult.style.marginTop = '10px';
iifResult.style.padding = '10px';
iifResult.style.border = '1px solid #ddd';
iifContainer.appendChild(iifResult);

// ボタンイベント
trueButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(true).subscribe(
    result => {
      iifResult.textContent = result;
      iifResult.style.color = 'green';
    }
  );
});

falseButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(false).subscribe(
    result => {
      iifResult.textContent = result;
      iifResult.style.color = 'red';
    }
  );
});
```

### ユースケース: EMPTYとの組み合わせ

`iif`はよく`EMPTY`と組み合わせて、条件を満たさない場合は何も発行しないObservableを作成するのに使われます。

```ts
import { iif, of, EMPTY } from 'rxjs';

// 条件を満たす場合のみ値を発行
function conditionalData(value: number) {
  return iif(
    () => value > 0,
    of(`正の値: ${value}`),
    EMPTY // 条件を満たさない場合は何も発行しない
  );
}

// UI要素を作成
const emptyContainer = document.createElement('div');
emptyContainer.innerHTML = '<h3>iif と EMPTY の組み合わせ:</h3>';
document.body.appendChild(emptyContainer);

// 入力フィールド
const valueInput = document.createElement('input');
valueInput.type = 'number';
valueInput.placeholder = '数値を入力';
valueInput.style.marginRight = '10px';
emptyContainer.appendChild(valueInput);

// 実行ボタン
const checkButton = document.createElement('button');
checkButton.textContent = '実行';
emptyContainer.appendChild(checkButton);

// 結果表示エリア
const emptyResult = document.createElement('div');
emptyResult.style.marginTop = '10px';
emptyResult.style.padding = '10px';
emptyResult.style.border = '1px solid #ddd';
emptyContainer.appendChild(emptyResult);

// ボタンイベント
checkButton.addEventListener('click', () => {
  const value = Number(valueInput.value);
  emptyResult.textContent = '';
  
  conditionalData(value).subscribe({
    next: result => {
      emptyResult.textContent = result;
      emptyResult.style.color = 'green';
    },
    complete: () => {
      if (!emptyResult.textContent) {
        emptyResult.textContent = '0以下の値が入力されたため、何も発行されませんでした';
        emptyResult.style.color = 'gray';
      }
    }
  });
});
```

## defer - 遅延評価によるObservable作成

`defer`オペレーターは、Observableのファクトリ関数を**購読時点**で実行し、その結果のObservableを返します。これにより、実際に購読されるまでObservableの作成を遅延させることができます。

```ts
import { defer, of } from 'rxjs';

// 現在時刻を返すObservable
const lazyDate$ = defer(() => {
  const currentDate = new Date();
  console.log('defer内部: 現在時刻を取得', currentDate);
  return of(currentDate);
});

// UI要素を作成
const deferContainer = document.createElement('div');
deferContainer.innerHTML = '<h3>defer オペレーターの例:</h3>';
document.body.appendChild(deferContainer);

// 購読ボタン
const subscribeButton = document.createElement('button');
subscribeButton.textContent = '購読';
deferContainer.appendChild(subscribeButton);

// 結果表示エリア
const deferResult = document.createElement('div');
deferResult.style.marginTop = '10px';
deferResult.style.padding = '10px';
deferResult.style.border = '1px solid #ddd';
deferContainer.appendChild(deferResult);

// ボタンイベント
subscribeButton.addEventListener('click', () => {
  console.log('ボタンクリック: lazyDate$を購読します');
  deferResult.textContent = '購読中...';
  
  lazyDate$.subscribe(date => {
    deferResult.textContent = `現在時刻: ${date.toLocaleTimeString()}`;
  });
});

// 説明テキスト
const explanation = document.createElement('p');
explanation.textContent = '「購読」ボタンをクリックするたびに、その時点の現在時刻が取得されます。deferは実際に購読されるまでObservableの作成を遅延させます。';
deferContainer.appendChild(explanation);
```

### ユースケース: ランダム値の生成

`defer`は副作用のある処理や毎回異なる結果を生成する処理に特に有用です。

```ts
import { defer, of } from 'rxjs';

// ランダムな数値を生成するObservable
const randomNumber$ = defer(() => {
  const random = Math.floor(Math.random() * 100);
  return of(random);
});

// UI要素を作成
const randomContainer = document.createElement('div');
randomContainer.innerHTML = '<h3>defer によるランダム値生成:</h3>';
document.body.appendChild(randomContainer);

// 生成ボタン
const generateButton = document.createElement('button');
generateButton.textContent = 'ランダム値を生成';
randomContainer.appendChild(generateButton);

// 履歴表示エリア
const randomHistory = document.createElement('div');
randomHistory.style.marginTop = '10px';
randomHistory.style.padding = '10px';
randomHistory.style.border = '1px solid #ddd';
randomHistory.style.maxHeight = '200px';
randomHistory.style.overflowY = 'auto';
randomContainer.appendChild(randomHistory);

// ボタンイベント
generateButton.addEventListener('click', () => {
  randomNumber$.subscribe(value => {
    const entry = document.createElement('div');
    entry.textContent = `生成された値: ${value}`;
    entry.style.padding = '5px';
    entry.style.margin = '2px 0';
    entry.style.backgroundColor = '#f5f5f5';
    entry.style.borderRadius = '3px';
    randomHistory.insertBefore(entry, randomHistory.firstChild);
  });
});

// 説明テキスト
const randomExplanation = document.createElement('p');
randomExplanation.textContent = '「ランダム値を生成」ボタンをクリックするたびに、新しいランダム値が生成されます。通常のofを使用した場合、値は最初の一度だけ生成されますが、deferを使用することで毎回新しい値を生成できます。';
randomContainer.appendChild(randomExplanation);
```

## defaultIfEmpty - ストリームが空の場合のデフォルト値

`defaultIfEmpty`オペレーターは、ソースObservableが値を発行せずに完了した場合に、指定したデフォルト値を発行します。

```ts
import { EMPTY, from } from 'rxjs';
import { defaultIfEmpty } from 'rxjs/operators';

// UI要素を作成
const defaultContainer = document.createElement('div');
defaultContainer.innerHTML = '<h3>defaultIfEmpty オペレーターの例:</h3>';
document.body.appendChild(defaultContainer);

// 空と非空の配列を切り替えるボタン
const emptyArrayButton = document.createElement('button');
emptyArrayButton.textContent = '空の配列を処理';
emptyArrayButton.style.marginRight = '10px';
defaultContainer.appendChild(emptyArrayButton);

const nonEmptyArrayButton = document.createElement('button');
nonEmptyArrayButton.textContent = '非空の配列を処理';
defaultContainer.appendChild(nonEmptyArrayButton);

// 結果表示エリア
const defaultResult = document.createElement('div');
defaultResult.style.marginTop = '10px';
defaultResult.style.padding = '10px';
defaultResult.style.border = '1px solid #ddd';
defaultContainer.appendChild(defaultResult);

// 空の配列を処理
emptyArrayButton.addEventListener('click', () => {
  defaultResult.textContent = '処理中...';
  
  from([])
    .pipe(
      defaultIfEmpty('データがありません')
    )
    .subscribe(value => {
      defaultResult.textContent = `結果: ${value}`;
    });
});

// 非空の配列を処理
nonEmptyArrayButton.addEventListener('click', () => {
  defaultResult.textContent = '処理中...';
  
  from([1, 2, 3])
    .pipe(
      defaultIfEmpty('データがありません')
    )
    .subscribe(value => {
      if (Array.isArray(value)) {
        defaultResult.textContent = `結果: ${value.join(', ')}`;
      } else {
        defaultResult.textContent = `結果: ${value}`;
      }
    });
});
```

### ユースケース: API結果が空の場合の処理

```ts
import { of, EMPTY } from 'rxjs';
import { defaultIfEmpty, delay } from 'rxjs/operators';

// APIレスポンスをシミュレート（成功/空）
function mockApiCall(hasData: boolean) {
  if (hasData) {
    return of([
      { id: 1, name: '項目1' },
      { id: 2, name: '項目2' },
      { id: 3, name: '項目3' }
    ]).pipe(delay(1000)); // 1秒の遅延
  } else {
    return EMPTY.pipe(delay(1000)); // 1秒の遅延
  }
}

// UI要素を作成
const apiContainer = document.createElement('div');
apiContainer.innerHTML = '<h3>API結果の defaultIfEmpty:</h3>';
document.body.appendChild(apiContainer);

// データ有無の切り替えボタン
const withDataButton = document.createElement('button');
withDataButton.textContent = 'データありの場合';
withDataButton.style.marginRight = '10px';
apiContainer.appendChild(withDataButton);

const noDataButton = document.createElement('button');
noDataButton.textContent = 'データなしの場合';
apiContainer.appendChild(noDataButton);

// 結果表示エリア
const apiResult = document.createElement('div');
apiResult.style.marginTop = '10px';
apiResult.style.padding = '10px';
apiResult.style.border = '1px solid #ddd';
apiContainer.appendChild(apiResult);

// データありの場合の処理
withDataButton.addEventListener('click', () => {
  apiResult.textContent = 'データを取得中...';
  withDataButton.disabled = true;
  noDataButton.disabled = true;
  
  mockApiCall(true)
    .pipe(
      defaultIfEmpty('データが見つかりませんでした')
    )
    .subscribe({
      next: value => {
        if (Array.isArray(value)) {
          apiResult.innerHTML = '<h4>取得結果:</h4>';
          
          const list = document.createElement('ul');
          value.forEach(item => {
            const listItem = document.createElement('li');
            listItem.textContent = `${item.id}: ${item.name}`;
            list.appendChild(listItem);
          });
          
          apiResult.appendChild(list);
        } else {
          apiResult.textContent = value;
        }
      },
      complete: () => {
        withDataButton.disabled = false;
        noDataButton.disabled = false;
      }
    });
});

// データなしの場合の処理
noDataButton.addEventListener('click', () => {
  apiResult.textContent = 'データを取得中...';
  withDataButton.disabled = true;
  noDataButton.disabled = true;
  
  mockApiCall(false)
    .pipe(
      defaultIfEmpty('データが見つかりませんでした')
    )
    .subscribe({
      next: value => {
        apiResult.innerHTML = value;
      },
      complete: () => {
        withDataButton.disabled = false;
        noDataButton.disabled = false;
      }
    });
});
```

## every - すべての値が条件を満たすか確認

`every`オペレーターは、ソースObservableから発行されるすべての値が指定した条件を満たすかどうかを判定します。結果は完了時に`true`または`false`として発行されます。

```ts
import { from } from 'rxjs';
import { every } from 'rxjs/operators';

// UI要素を作成
const everyContainer = document.createElement('div');
everyContainer.innerHTML = '<h3>every オペレーターの例:</h3>';
document.body.appendChild(everyContainer);

// テスト条件の説明
const everyExplanation = document.createElement('p');
everyExplanation.textContent = '条件: すべての値が偶数かどうか';
everyContainer.appendChild(everyExplanation);

// テストケースボタン
const allEvenButton = document.createElement('button');
allEvenButton.textContent = '偶数のみ [2, 4, 6, 8]';
allEvenButton.style.marginRight = '10px';
everyContainer.appendChild(allEvenButton);

const someOddButton = document.createElement('button');
someOddButton.textContent = '奇数を含む [2, 4, 5, 8]';
everyContainer.appendChild(someOddButton);

// 結果表示エリア
const everyResult = document.createElement('div');
everyResult.style.marginTop = '10px';
everyResult.style.padding = '10px';
everyResult.style.border = '1px solid #ddd';
everyContainer.appendChild(everyResult);

// すべて偶数の場合
allEvenButton.addEventListener('click', () => {
  everyResult.textContent = '判定中...';
  
  const numbers = [2, 4, 6, 8];
  
  from(numbers)
    .pipe(
      every(num => num % 2 === 0) // すべての値が偶数か
    )
    .subscribe(result => {
      everyResult.textContent = `すべて偶数?: ${result}`;
      everyResult.style.color = result ? 'green' : 'red';
    });
});

// 奇数を含む場合
someOddButton.addEventListener('click', () => {
  everyResult.textContent = '判定中...';
  
  const numbers = [2, 4, 5, 8];
  
  from(numbers)
    .pipe(
      every(num => num % 2 === 0) // すべての値が偶数か
    )
    .subscribe(result => {
      everyResult.textContent = `すべて偶数?: ${result}`;
      everyResult.style.color = result ? 'green' : 'red';
    });
});
```

### ユースケース: フォームのバリデーション

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith, every } from 'rxjs/operators';

// UI要素を作成
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>フォームバリデーション with every:</h3>';
document.body.appendChild(formContainer);

// フォーム作成
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formContainer.appendChild(form);

// 名前入力
const nameLabel = document.createElement('label');
nameLabel.textContent = '名前: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.width = '100%';
nameInput.style.padding = '5px';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

// 年齢入力
const ageLabel = document.createElement('label');
ageLabel.textContent = '年齢: ';
ageLabel.style.display = 'block';
ageLabel.style.marginBottom = '5px';
form.appendChild(ageLabel);

const ageInput = document.createElement('input');
ageInput.type = 'number';
ageInput.min = '0';
ageInput.style.width = '100%';
ageInput.style.padding = '5px';
ageInput.style.marginBottom = '15px';
form.appendChild(ageInput);

// メール入力
const emailLabel = document.createElement('label');
emailLabel.textContent = 'メール: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.width = '100%';
emailInput.style.padding = '5px';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

// 送信ボタン
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = '送信';
submitButton.disabled = true;
form.appendChild(submitButton);

// バリデーションメッセージ
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// 名前のバリデーション
const nameValid$ = fromEvent(nameInput, 'input').pipe(
  map(event => {
    const value = (event.target as HTMLInputElement).value.trim();
    return value.length >= 2;
  }),
  startWith(false)
);

// 年齢のバリデーション
const ageValid$ = fromEvent(ageInput, 'input').pipe(
  map(event => {
    const value = Number((event.target as HTMLInputElement).value);
    return !isNaN(value) && value > 0 && value < 120;
  }),
  startWith(false)
);

// メールのバリデーション
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const emailValid$ = fromEvent(emailInput, 'input').pipe(
  map(event => {
    const value = (event.target as HTMLInputElement).value.trim();
    return emailRegex.test(value);
  }),
  startWith(false)
);

// すべてのフィールドのバリデーション
combineLatest([nameValid$, ageValid$, emailValid$])
  .pipe(
    every(valid => valid === true)
  )
  .subscribe(allValid => {
    submitButton.disabled = !allValid;
    
    if (allValid) {
      validationMessage.textContent = '';
    } else {
      validationMessage.textContent = 'すべてのフィールドを正しく入力してください';
    }
  });

// フォーム送信
form.addEventListener('submit', event => {
  event.preventDefault();
  
  const formData = {
    name: nameInput.value,
    age: ageInput.value,
    email: emailInput.value
  };
  
  validationMessage.textContent = 'フォームが送信されました！';
  validationMessage.style.color = 'green';
  
  console.log('送信データ:', formData);
});
```

## isEmpty - ストリームが空かどうかを判定

`isEmpty`オペレーターは、ソースObservableが値を発行せずに完了するかどうかを判定します。結果は単一の真偽値として発行されます。

```ts
import { EMPTY, of } from 'rxjs';
import { isEmpty } from 'rxjs/operators';

// UI要素を作成
const isEmptyContainer = document.createElement('div');
isEmptyContainer.innerHTML = '<h3>isEmpty オペレーターの例:</h3>';
document.body.appendChild(isEmptyContainer);

// テストケースボタン
const emptyObservableButton = document.createElement('button');
emptyObservableButton.textContent = '空のObservable (EMPTY)';
emptyObservableButton.style.marginRight = '10px';
isEmptyContainer.appendChild(emptyObservableButton);

const nonEmptyObservableButton = document.createElement('button');
nonEmptyObservableButton.textContent = '非空のObservable (of(1, 2, 3))';
isEmptyContainer.appendChild(nonEmptyObservableButton);

// 結果表示エリア
const isEmptyResult = document.createElement('div');
isEmptyResult.style.marginTop = '10px';
isEmptyResult.style.padding = '10px';
isEmptyResult.style.border = '1px solid #ddd';
isEmptyContainer.appendChild(isEmptyResult);

// 空のObservableのテスト
emptyObservableButton.addEventListener('click', () => {
  isEmptyResult.textContent = 'チェック中...';
  
  EMPTY
    .pipe(
      isEmpty()
    )
    .subscribe(result => {
      isEmptyResult.textContent = `空ですか？: ${result}`;
      isEmptyResult.style.color = result ? 'blue' : 'purple';
    });
});

// 非空のObservableのテスト
nonEmptyObservableButton.addEventListener('click', () => {
  isEmptyResult.textContent = 'チェック中...';
  
  of(1, 2, 3)
    .pipe(
      isEmpty()
    )
    .subscribe(result => {
      isEmptyResult.textContent = `空ですか？: ${result}`;
      isEmptyResult.style.color = result ? 'blue' : 'purple';
    });
});
```

## 実践的なユースケース

### 条件に基づく異なるデータソースの選択

```ts
import { iif, from, of, EMPTY } from 'rxjs';
import { switchMap, tap, catchError, retry } from 'rxjs/operators';

// UIの作成
const appContainer = document.createElement('div');
appContainer.innerHTML = '<h3>データソース選択アプリ:</h3>';
document.body.appendChild(appContainer);

// オプション選択
const optionsDiv = document.createElement('div');
optionsDiv.style.marginBottom = '15px';
appContainer.appendChild(optionsDiv);

// チェックボックス（オフラインモード）
const offlineCheck = document.createElement('input');
offlineCheck.type = 'checkbox';
offlineCheck.id = 'offlineMode';
optionsDiv.appendChild(offlineCheck);

const offlineLabel = document.createElement('label');
offlineLabel.htmlFor = 'offlineMode';
offlineLabel.textContent = 'オフラインモード';
offlineLabel.style.marginLeft = '5px';
optionsDiv.appendChild(offlineLabel);

// 検索ID入力
const idInput = document.createElement('input');
idInput.type = 'number';
idInput.placeholder = 'ID (1-10)';
idInput.min = '1';
idInput.max = '10';
idInput.style.marginLeft = '15px';
idInput.style.width = '80px';
optionsDiv.appendChild(idInput);

// 検索ボタン
const searchButton = document.createElement('button');
searchButton.textContent = '検索';
searchButton.style.marginLeft = '10px';
optionsDiv.appendChild(searchButton);

// 結果エリア
const resultsArea = document.createElement('div');
resultsArea.style.padding = '15px';
resultsArea.style.border = '1px solid #ddd';
resultsArea.style.borderRadius = '5px';
resultsArea.style.backgroundColor = '#f9f9f9';
resultsArea.style.minHeight = '150px';
appContainer.appendChild(resultsArea);

// オフラインデータ（キャッシュ）
const cachedData = {
  1: { id: 1, name: '山田太郎', email: 'yamada@example.com' },
  2: { id: 2, name: '佐藤花子', email: 'sato@example.com' },
  3: { id: 3, name: '鈴木一郎', email: 'suzuki@example.com' }
};

// オンラインAPIシミュレーション
function fetchUserFromApi(id) {
  console.log(`APIからユーザーID ${id} を取得中...`);
  
  // 50%の確率で失敗するAPI
  const shouldFail = Math.random() < 0.5;
  
  if (shouldFail) {
    return of(null).pipe(
      tap(() => console.log('API呼び出しに失敗')),
      switchMap(() => EMPTY.pipe(
        tap(() => { throw new Error('ネットワークエラー'); })
      )),
      catchError(err => {
        throw new Error('APIリクエストに失敗しました');
      })
    );
  }
  
  // 成功時は1秒後にデータを返す
  return of({
    id: id,
    name: `オンラインユーザー${id}`,
    email: `user${id}@example.com`,
    lastUpdated: new Date().toISOString()
  }).pipe(
    tap(() => console.log('API呼び出しに成功')),
    // 1秒の遅延をシミュレート
    switchMap(data => {
      return new Promise(resolve => {
        setTimeout(() => resolve(data), 1000);
      });
    })
  );
}

// キャッシュからユーザーを取得
function getUserFromCache(id) {
  console.log(`キャッシュからユーザーID ${id} を取得中...`);
  
  return iif(
    () => id in cachedData,
    of({ ...cachedData[id], fromCache: true }),
    EMPTY.pipe(
      tap(() => { throw new Error('キャッシュにユーザーが見つかりません'); })
    )
  );
}

// 検索ボタンクリック
searchButton.addEventListener('click', () => {
  const id = parseInt(idInput.value, 10);
  const isOffline = offlineCheck.checked;
  
  // 入力検証
  if (isNaN(id) || id < 1 || id > 10) {
    resultsArea.innerHTML = '<p style="color: red;">有効なID (1-10) を入力してください</p>';
    return;
  }
  
  // ローディング表示
  resultsArea.innerHTML = '<p>データを取得中...</p>';
  
  // オフラインモードに基づいてデータソースを選択
  iif(
    () => isOffline,
    getUserFromCache(id).pipe(
      catchError(err => {
        console.error('キャッシュエラー:', err);
        return of({ error: err.message });
      })
    ),
    fetchUserFromApi(id).pipe(
      retry(2), // 最大2回再試行
      catchError(err => {
        console.error('APIエラー:', err);
        
        // APIが失敗したらキャッシュをフォールバックとして使用
        return getUserFromCache(id).pipe(
          catchError(() => of({ error: 'オンラインAPIとキャッシュの両方が失敗しました' }))
        );
      })
    )
  ).subscribe({
    next: result => {
      if ('error' in result) {
        // エラー表示
        resultsArea.innerHTML = `
          <p style="color: red;">エラー: ${result.error}</p>
        `;
      } else {
        // データ表示
        const source = result.fromCache ? 
          '<span style="color: orange;">(キャッシュから)</span>' : 
          '<span style="color: green;">(APIから)</span>';
        
        resultsArea.innerHTML = `
          <h4>ユーザー情報 ${source}</h4>
          <p><strong>ID:</strong> ${result.id}</p>
          <p><strong>名前:</strong> ${result.name}</p>
          <p><strong>メール:</strong> ${result.email}</p>
          ${result.lastUpdated ? 
            `<p><small>最終更新: ${new Date(result.lastUpdated).toLocaleString()}</small></p>` : 
            ''}
        `;
      }
    },
    error: err => {
      // エラー表示
      resultsArea.innerHTML = `
        <p style="color: red;">エラー: ${err.message}</p>
      `;
    }
  });
});

// 初期メッセージ
resultsArea.innerHTML = '<p>ボタンをクリックしてデータを取得してください</p>';
```

### パフォーマンス最適化パターン

より複雑なシナリオでは、条件演算子を組み合わせて最適化されたデータ取得パターンを実装できます。

```ts
import { fromEvent, of, EMPTY, timer, combineLatest } from 'rxjs';
import { switchMap, catchError, map, tap, debounceTime, filter, distinctUntilChanged, withLatestFrom, shareReplay } from 'rxjs/operators';

// UI要素の作成
const optimizationContainer = document.createElement('div');
optimizationContainer.innerHTML = '<h3>高度な条件付きデータ取得:</h3>';
document.body.appendChild(optimizationContainer);

// 検索UI
const searchInputGroup = document.createElement('div');
searchInputGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(searchInputGroup);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'ユーザーIDを入力 (1-10)';
searchInput.style.padding = '8px';
searchInput.style.width = '180px';
searchInputGroup.appendChild(searchInput);

const searchButton = document.createElement('button');
searchButton.textContent = '検索';
searchButton.style.marginLeft = '10px';
searchButton.style.padding = '8px 16px';
searchInputGroup.appendChild(searchButton);

// オプション設定
const optionsGroup = document.createElement('div');
optionsGroup.style.marginBottom = '15px';
optimizationContainer.appendChild(optionsGroup);

const cacheCheckbox = document.createElement('input');
cacheCheckbox.type = 'checkbox';
cacheCheckbox.id = 'useCache';
cacheCheckbox.checked = true;
optionsGroup.appendChild(cacheCheckbox);

const cacheLabel = document.createElement('label');
cacheLabel.htmlFor = 'useCache';
cacheLabel.textContent = 'キャッシュを使用';
cacheLabel.style.marginRight = '15px';
optionsGroup.appendChild(cacheLabel);

const forceCheckbox = document.createElement('input');
forceCheckbox.type = 'checkbox';
forceCheckbox.id = 'forceRefresh';
optionsGroup.appendChild(forceCheckbox);

const forceLabel = document.createElement('label');
forceLabel.htmlFor = 'forceRefresh';
forceLabel.textContent = '強制再取得';
optionsGroup.appendChild(forceLabel);

// 結果表示エリア
const optimizedResults = document.createElement('div');
optimizedResults.style.padding = '15px';
optimizedResults.style.border = '1px solid #ddd';
optimizedResults.style.borderRadius = '5px';
optimizedResults.style.minHeight = '150px';
optimizedResults.style.backgroundColor = '#f9f9f9';
optimizationContainer.appendChild(optimizedResults);

// キャッシュ管理
const cache = new Map<string, { data: any, timestamp: number }>();
const CACHE_EXPIRY = 30000; // 30秒

// ユーザーデータをシミュレート取得するAPI
function fetchUserData(id: string, forceRefresh: boolean): Observable<any> {
  // 無効なID
  if (!id || isNaN(Number(id)) || Number(id) < 1 || Number(id) > 10) {
    return throwError(() => new Error('無効なユーザーID: 1～10の数値を入力してください'));
  }
  
  const cacheKey = `user-${id}`;
  const cachedItem = cache.get(cacheKey);
  const now = Date.now();
  
  // キャッシュチェック（期限内かつ強制再取得でない場合）
  if (!forceRefresh && cachedItem && (now - cachedItem.timestamp < CACHE_EXPIRY)) {
    console.log(`キャッシュから取得: ${id}`);
    return of({
      ...cachedItem.data,
      fromCache: true
    }).pipe(delay(100)); // 高速レスポンスをシミュレート
  }
  
  // APIリクエストをシミュレート
  console.log(`APIからデータ取得: ${id}`);
  
  // 25%の確率でランダムにエラー発生
  const shouldFail = Math.random() < 0.25;
  
  if (shouldFail) {
    return timer(1500).pipe(
      switchMap(() => throwError(() => new Error('APIリクエストに失敗しました')))
    );
  }
  
  // 成功レスポンス
  return timer(1500).pipe(
    map(() => {
      const userData = {
        id: Number(id),
        name: `ユーザー${id}`,
        email: `user${id}@example.com`,
        lastUpdated: now,
        fromCache: false
      };
      
      // キャッシュに保存
      cache.set(cacheKey, {
        data: userData,
        timestamp: now
      });
      
      return userData;
    })
  );
}

// 検索条件の変更を監視
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  debounceTime(300),
  distinctUntilChanged()
);

// キャッシュ設定の変更を監視
const useCache$ = fromEvent(cacheCheckbox, 'change').pipe(
  map(event => (event.target as HTMLInputElement).checked),
  startWith(true)
);

// 強制再取得の変更を監視
const forceRefresh$ = fromEvent(forceCheckbox, 'change').pipe(
  map(event => (event.target as HTMLInputElement).checked),
  startWith(false)
);

// 検索ボタンのクリックイベント
const searchClick$ = fromEvent(searchButton, 'click');

// 検索実行
searchClick$.pipe(
  // 現在の入力値、キャッシュ設定、強制再取得設定を取得
  withLatestFrom(
    searchTerm$,
    useCache$,
    forceRefresh$,
    (_, term, useCache, forceRefresh) => ({
      term,
      useCache,
      forceRefresh
    })
  ),
  tap(() => {
    // 検索開始表示
    optimizedResults.innerHTML = '<p>検索中...</p>';
  }),
  // iif()を使用した条件付きストリーム
  switchMap(({ term, useCache, forceRefresh }) => {
    // 検索語が空の場合
    if (!term) {
      return of({ error: '検索語を入力してください' });
    }
    
    // キャッシュ無効の場合
    if (!useCache) {
      return fetchUserData(term, true);
    }
    
    // 通常の検索（キャッシュ使用＆必要に応じて強制再取得）
    return fetchUserData(term, forceRefresh);
  }),
  // エラー処理
  catchError(err => {
    return of({ error: err.message });
  })
).subscribe({
  next: result => {
    if ('error' in result) {
      // エラー表示
      optimizedResults.innerHTML = `
        <p style="color: red;">エラー: ${result.error}</p>
      `;
    } else {
      // データ表示
      const source = result.fromCache ? 
        '<span style="color: orange;">(キャッシュから)</span>' : 
        '<span style="color: green;">(APIから)</span>';
      
      optimizedResults.innerHTML = `
        <h4>ユーザー情報 ${source}</h4>
        <p><strong>ID:</strong> ${result.id}</p>
        <p><strong>名前:</strong> ${result.name}</p>
        <p><strong>メール:</strong> ${result.email}</p>
        ${result.lastUpdated ?
          `<p><small>最終更新: ${new Date(result.lastUpdated).toLocaleString()}</small></p>` : 
          ''}
      `;
    }
  }
});

// 初期メッセージ
optimizedResults.innerHTML = '<p>ユーザーIDを入力して検索ボタンをクリックしてください</p>';
```

## 条件オペレーターの選び方

| オペレーター | ユースケース | 特徴 |
|------------|------------|------|
| `iif` | 実行時に1つのストリームを選択する | 2つの選択肢から条件に基づいて1つを選択 |
| `partition` | ストリームを条件で2つに分ける | 元ストリームを条件でTrue/Falseに分割 |
| `throwIfEmpty` | 空のストリームを検出する | 値が一つも発行されない場合にエラーを投げる |
| `defaultIfEmpty` | 空の場合にデフォルト値を使用 | ストリームが空の場合のフォールバック値を提供 |

### 選択の判断フロー

1. **選択肢が2つあるか？**
   - Yes → `iif`を使用
   - No → 次へ

2. **ストリームを分割したいか？**
   - Yes → `partition`を使用
   - No → 次へ

3. **空ストリームに対処したいか？**
   - Yes → 空ストリームをエラーとして扱いたいか？
     - Yes → `throwIfEmpty`
     - No → `defaultIfEmpty`
   - No → 次へ

4. **単純に条件に基づいて値をフィルタリングしたいか？**
   - Yes → `filter`オペレーターを使用（基本的なフィルタリングオペレーター）
   - No → 目的を再検討

## まとめ

条件オペレーターは、ストリームのフローを制御し、特定の条件に基づいて処理を分岐させるための強力なツールです。主なポイントは次のとおりです：

1. **決定に基づくリアクティブフロー**：条件オペレーターを使用することで、イベントやデータの状態に応じて処理を動的に変更できます。

2. **エラー処理の強化**：条件オペレーターは、エラー処理戦略の重要な部分として機能し、例外ケースに対するグレースフルな対応を可能にします。

3. **最適化の機会**：条件付き実行により、不必要な処理を回避し、特にネットワークリクエストやハードウェアアクセスといったコストの高い操作を最適化できます。

4. **複雑なアプリケーションフロー**：複数の条件オペレーターを組み合わせることで、複雑なビジネスロジックや状態管理を宣言的に表現できます。

条件オペレーターは、RxJSを使用したエラー処理、キャッシュ戦略、フォールバックメカニズム、および条件付き実行パターンを実装する際に特に価値を発揮します。他のオペレーターと組み合わせることで、宣言的かつ型安全な方法で複雑なアプリケーションフローを構築できます。