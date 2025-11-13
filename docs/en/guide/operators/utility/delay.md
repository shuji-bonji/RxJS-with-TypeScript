---
description: The delay operator delays the emission timing of each value in the Observable by a specified amount of time, making it effective for UI direction and asynchronous control.
---

# delay - Value Delay

The `delay` operator is used to delay the emission of each value in a stream by a specified amount of time.
This is useful for staging animations and adjusting the timing of feedback display to the user.


## 🔰 Basic Syntax and Operation

This is the minimum configuration to emit a value after a certain time.

```ts
import { of } from 'rxjs';
import { delay } from 'rxjs';

of('Hello')
  .pipe(
    delay(1000) // Emit value after 1 second
  )
  .subscribe(console.log);
// Output:
// Hello
```

In this example, the value created by `of('Hello')` is received by `subscribe()` with a 1 second delay.

[🌐 RxJS Official Documentation - delay](https://rxjs.dev/api/index/function/delay)

## 💡 Typical Usage Example

This is an example of using delay to adjust the timing of emission in a situation where multiple values are emitted.

```ts
import { of } from 'rxjs';
import { delay, concatMap } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    concatMap(
      (val, index) => of(val).pipe(delay(1000 * index)) // A immediately, B after 1 second, C after 2 seconds
    )
  )
  .subscribe(console.log);
// Output:
// A
// B
```

In this way, it is also possible to set a separate delay for each value by combining it with `concatMap`.


## 🧪 Practical Code Example (with UI)

```ts
import { of } from 'rxjs';
import { delay, tap } from 'rxjs';

// Output display area
const delayOutput = document.createElement('div');
delayOutput.innerHTML = '<h3>delay Example:</h3>';
document.body.appendChild(delayOutput);

// Function to display current time
function addTimeLog(message: string) {
  const now = new Date();
  const time =
    now.toLocaleTimeString('en-US', { hour12: false }) +
    '.' +
    now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `${time}: ${message}`;
  delayOutput.appendChild(logItem);
}

// Record start time
addTimeLog('Start');

// Value sequence
of('A', 'B', 'C')
  .pipe(
    tap((val) => addTimeLog(`Before value ${val} is emitted`)),
    delay(1000), // 1 second delay
    tap((val) => addTimeLog(`Value ${val} emitted after 1 second`))
  )
  .subscribe();
```


## ✅ Summary

- `delay` is an operator for **controlling the timing of Observable output**
- Can be combined with `concatMap` to **control delay per value**
- Useful for **asynchronous adjustments** to improve UX, such as output to UI and timer direction
