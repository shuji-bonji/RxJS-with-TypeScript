---
description: "everyオペレーターは、すべての値が指定条件を満たすかを評価し、最初に条件を満たさなかった時点でfalseを返す短絡的な判定が可能です。バリデーション、データ品質チェック、配列のArray.every()に相当するストリーム処理をTypeScriptで型安全に実装します。"
---

# every - すべての値が条件を満たすか確認

`every` オペレーターは、ソースObservableから発行されるすべての値が指定した条件を満たすかどうかを評価し、  
**最初に条件を満たさなかった時点で`false`を返して終了**します。すべて満たせば`true`が返ります。

## 🔰 基本構文・動作

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 6, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// 出力: true
```

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 5, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// 出力: false（5で止まる）
```

[🌐 RxJS公式ドキュメント - every](https://rxjs.dev/api/index/function/every)

## 💡 典型的な活用例

- **バリデーションチェック**：すべての条件が満たされたか確認
- **一括入力の検証**：複数値をまとめて評価する場面
- **配列のフィルターと異なり、全体の満足度を一発で確認**する場合に有効

## 🧪 実践コード例（UI付き）

### ✅ 1. 配列がすべて偶数かを判定する

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>every オペレーターの例:</h3>';
document.body.appendChild(container);

const allEvenButton = document.createElement('button');
allEvenButton.textContent = '偶数のみ [2, 4, 6, 8]';
container.appendChild(allEvenButton);

const someOddButton = document.createElement('button');
someOddButton.textContent = '奇数を含む [2, 4, 5, 8]';
container.appendChild(someOddButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

allEvenButton.addEventListener('click', () => {
  result.textContent = '判定中...';
  from([2, 4, 6, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `すべて偶数？: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});

someOddButton.addEventListener('click', () => {
  result.textContent = '判定中...';
  from([2, 4, 5, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `すべて偶数？: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});
```

### ✅ 2. フォームバリデーションでの活用

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith, every, tap } from 'rxjs';

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
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return value.length >= 2;
  }),
  startWith(false)
);

// 年齢のバリデーション
const ageValid$ = fromEvent(ageInput, 'input').pipe(
  map((event) => {
    const value = Number((event.target as HTMLInputElement).value);
    return !isNaN(value) && value > 0 && value < 120;
  }),
  startWith(false)
);

// メールのバリデーション
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const emailValid$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return emailRegex.test(value);
  }),
  startWith(false)
);

// すべてのフィールドのバリデーション
combineLatest([nameValid$, ageValid$, emailValid$])
  .pipe(
    // tap((v) => console.log(v)),
    map((validList) => validList.every((v) => v === true))
  )
  .subscribe((allValid) => {
    submitButton.disabled = !allValid;
    if (allValid) {
      validationMessage.textContent = '';
    } else {
      validationMessage.textContent =
        'すべてのフィールドを正しく入力してください';
    }
  });

// フォーム送信
form.addEventListener('submit', (event) => {
  event.preventDefault();

  const formData = {
    name: nameInput.value,
    age: ageInput.value,
    email: emailInput.value,
  };

  validationMessage.textContent = 'フォームが送信されました！';
  validationMessage.style.color = 'green';

  console.log('送信データ:', formData);
});

```