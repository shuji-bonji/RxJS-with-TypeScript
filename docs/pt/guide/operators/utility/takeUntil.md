---
description: The takeUntil operator is used to subscribe to the original Observable until the notifier Observable emits a value and then unsubscribe when notified.
---

# takeUntil

The `takeUntil` operator **keeps subscribing to the original Observable until the specified Observable (notification trigger) emits its first value**. The original Observable is unsubscribed at the time the notification trigger emits.

## 🔁 Basic Syntax

```ts
source$.pipe(
  takeUntil(notifier$)
)
```

- `source$`: Original Observable (subscription target)
- `notifier$`: Observable that signals stop (subscription stops when this Observable emits its first value)

[🌐 RxJS Official Documentation - takeUntil](https://rxjs.dev/api/index/function/takeUntil)

## 🧪 Usage Example: Stop Subscription on Button Click

```ts
import { interval, fromEvent } from 'rxjs';
import { takeUntil } from 'rxjs';

const stopButton = document.createElement('button');
stopButton.textContent = 'stop';
document.body.appendChild(stopButton)

const stop$ = fromEvent(stopButton, 'click');
const source$ = interval(1000); // Emit number every second

source$
  .pipe(takeUntil(stop$))
  .subscribe((val) => console.log(`Value: ${val}`));
```

📌 When `stopButton` is clicked, the subscription to `source$` stops at that moment.

## ✅ Common Use Cases

- When you want to stop HTTP requests or polling process with a cancel button
- When you want to unsubscribe a component according to its lifecycle
- When you want to terminate asynchronous processing by page transition or unmounting

## 🔗 Related Operators

- `take`: Take values up to a certain number of times
- `first`: Fetch only the first case and exit
- `skipUntil`: Ignore until a particular Observable emits a value
