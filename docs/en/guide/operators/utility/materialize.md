---
description: materialize is an RxJS utility operator that converts Observable notifications (next, error, complete) into Notification objects. It is ideal for situations where you want to manipulate the notification itself, such as handling errors as data, debugging and logging notifications, recording meta-information, etc. dematerialize allows you to restore the original format and type-safe notification processing with TypeScript type inference.
---

# materialize - Objectize Notifications

The `materialize` operator converts Observable **notifications (next, error, complete) into Notification objects**. This allows not only values but also errors and completions to be handled as data.

## 🔰 Basic Syntax and Operation

Converts a normal stream into a stream of Notification objects.

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

of(1, 2, 3)
  .pipe(materialize())
  .subscribe(notification => {
    console.log(notification);
  });
// Output:
// Notification { kind: 'N', value: 1, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 2, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 3, error: undefined, hasValue: true }
// Notification { kind: 'C', value: undefined, error: undefined, hasValue: false }
```

The `kind` property of the Notification object:
- `'N'`: next (value issued)
- `'E'`: error
- `'C'`: complete

[🌐 RxJS Official Documentation - materialize](https://rxjs.dev/api/index/function/materialize)

## 💡 Typical Usage Examples

- **Error datamining**: Treat errors as part of the stream
- **Debugging and logging**: Detailed tracking of notifications
- **Recording meta-information**: Record when and what kind of notifications occur
- **Combining streams with errors**: Handle errors in multiple streams in a unified manner

## 🧪 Practical Code Example 1: Treat Errors as Data

This example shows how to treat errors that would normally interrupt a stream as data and continue.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, map } from 'rxjs';

// UI creation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'materialize - Error datamining';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.appendChild(logItem);
}

// Normal error handling (stream interrupted)
addLog('--- Normal error handling ---', '#e3f2fd');
concat(
  of(1, 2),
  throwError(() => new Error('Error occurred')),
  of(3, 4)  // Not executed here
).subscribe({
  next: v => addLog(`Value: ${v}`, '#c8e6c9'),
  error: err => addLog(`❌ Error: ${err.message}`, '#ffcdd2'),
  complete: () => addLog('Completed', '#e3f2fd')
});

// Using materialize (stream continues)
setTimeout(() => {
  addLog('--- Using materialize ---', '#e3f2fd');

  concat(
    of(1, 2),
    throwError(() => new Error('Error occurred')),
    of(3, 4)
  )
    .pipe(
      materialize(),
      map(notification => {
        if (notification.kind === 'N') {
          return `Value: ${notification.value}`;
        } else if (notification.kind === 'E') {
          return `Error (datamined): ${notification.error?.message}`;
        } else {
          return 'Completed';
        }
      })
    )
    .subscribe({
      next: msg => {
        const color = msg.includes('Error') ? '#fff9c4' : '#c8e6c9';
        addLog(msg, color);
      },
      complete: () => addLog('Stream completed', '#e3f2fd')
    });
}, 1000);
```

- Normal errors interrupt the stream
- With `materialize`, errors are treated as data and the stream continues

## 🧪 Practical Code Example 2: Debug Logging

Here is an example that logs out all notifications in detail.

```ts
import { interval, throwError } from 'rxjs';
import { materialize, take, mergeMap } from 'rxjs';

// UI creation
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'materialize - Debug logging';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '250px';
output2.style.overflow = 'auto';
output2.style.fontFamily = 'monospace';
output2.style.fontSize = '12px';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('en-US', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.marginBottom = '2px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

interval(500)
  .pipe(
    take(5),
    mergeMap(value => {
      // Generate error when value is 3
      if (value === 3) {
        return throwError(() => new Error('Error at value 3'));
      }
      return of(value);
    }),
    materialize()
  )
  .subscribe({
    next: notification => {
      switch (notification.kind) {
        case 'N':
          addLog2(`[NEXT] value: ${notification.value}`);
          break;
        case 'E':
          addLog2(`[ERROR] ${notification.error?.message}`);
          break;
        case 'C':
          addLog2('[COMPLETE]');
          break;
      }
    },
    complete: () => {
      addLog2('--- Observer completed ---');
    }
  });
```

- Uniform logging of all notification types (next, error, complete)
- Tracks the order in which notifications occur with timestamps
- Useful for debugging and monitoring

## 🆚 Comparison with Normal Streams

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

// Normal stream
of(1, 2, 3).subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('Completed')
});
// Output:
// Value: 1
// Value: 2
// Value: 3
// Completed

// Using materialize
of(1, 2, 3)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notification:', n),
    complete: () => console.log('Completed')
  });
// Output:
// Notification: Notification { kind: 'N', value: 1, ... }
// Notification: Notification { kind: 'N', value: 2, ... }
// Notification: Notification { kind: 'N', value: 3, ... }
// Notification: Notification { kind: 'C', ... }
// Completed
```

## Manipulate Notification Object

```ts
import { of } from 'rxjs';
import { materialize, map } from 'rxjs';

of(10, 20, 30)
  .pipe(
    materialize(),
    map(notification => {
      // Properties of Notification object
      return {
        kind: notification.kind,           // 'N', 'E', 'C'
        hasValue: notification.hasValue,   // Has value
        value: notification.value,         // Value (for next)
        error: notification.error          // Error (for error)
      };
    })
  )
  .subscribe(console.log);
// Output:
// { kind: 'N', hasValue: true, value: 10, error: undefined }
// { kind: 'N', hasValue: true, value: 20, error: undefined }
// { kind: 'N', hasValue: true, value: 30, error: undefined }
// { kind: 'C', hasValue: false, value: undefined, error: undefined }
```

## ⚠️ Important Notes

### 1. Errors Do Not Interrupt the Stream

When using `materialize`, errors are treated as data and the stream is not interrupted.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize } from 'rxjs';

concat(
  of(1),
  throwError(() => new Error('Error')),
  of(2)
)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notification:', n.kind),
    error: () => console.log('Error handler'),  // Not called
    complete: () => console.log('Completed')
  });
// Output:
// Notification: N
// Notification: E  ← Errors are also treated as next
// Completed
```

### 2. Combination with dematerialize

Streams transformed with `materialize` can be restored with `dematerialize`.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),
    // Some processing here
    dematerialize()  // Restore
  )
  .subscribe(console.log);
// Output: 1, 2, 3
```

### 3. Performance Impact

There is an overhead in generating Notification objects. Use only when necessary in a production environment.

## 📚 Related Operators

- **[dematerialize](./dematerialize)** - Revert Notification object to normal notification
- **[tap](./tap)** - Perform a side effect (for debugging purposes)
- **[catchError](/en/guide/error-handling/retry-catch)** - Error handling

## ✅ Summary

The `materialize` operator converts a notification into a Notification object.

- ✅ Can handle errors as data
- ✅ Useful for debugging and logging
- ✅ Can record meta-information about notifications
- ✅ Can be undone with `dematerialize`
- ⚠️ Errors will no longer interrupt the stream
- ⚠️ Note performance overhead
