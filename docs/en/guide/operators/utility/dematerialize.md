---
description: dematerialize is a RxJS utility operator that restores Notification objects to normal notifications (next, error, complete) and performs the reverse transformation of materialize. It is ideal for restoring notifications after processing, filtering or converting errors, reordering or buffering notifications, or any other situation where you want to process notifications as data and then return them to their original format.
---

# dematerialize - Restore Notification Object

The `dematerialize` operator **converts** a Notification object into a normal notification (next, error, complete). It performs the reverse transformation of `materialize`, restoring the datatized notification to its original form.

## 🔰 Basic Syntax and Operation

Converts a stream of Notification objects back to a normal stream.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),     // Convert to Notification object
    dematerialize()    // Restore
  )
  .subscribe({
    next: v => console.log('Value:', v),
    complete: () => console.log('Completed')
  });
// Output:
// Value: 1
// Value: 2
// Value: 3
// Completed
```

[🌐 RxJS Official Documentation - dematerialize](https://rxjs.dev/api/index/function/dematerialize)

## 💡 Typical Usage Examples

- **Restore notifications after processing**: Restore them to their original format after processing with materialize
- **Filtering errors**: Exclude only certain errors
- **Rearranging the order of notifications**: Restore after sorting notifications as data
- **Restore after debugging**: Restore normal operation after logging, etc.

## 🧪 Practical Code Example 1: Selective Filtering of Errors

This is an example of excluding only certain errors and processing the rest as normal.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize, filter } from 'rxjs';

// UI creation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'dematerialize - Error filtering';
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

// Stream with errors
const source$ = concat(
  of(1, 2),
  throwError(() => new Error('Ignorable error')),
  of(3, 4),
  throwError(() => new Error('Critical error')),
  of(5)
);

source$
  .pipe(
    materialize(),
    filter(notification => {
      // Filter only "Ignorable error"
      if (notification.kind === 'E') {
        const errorMessage = notification.error?.message || '';
        if (errorMessage.includes('Ignorable')) {
          addLog(`🔇 Ignored: ${errorMessage}`, '#fff9c4');
          return false;  // Exclude this error
        }
      }
      return true;
    }),
    dematerialize()  // Restore to original format
  )
  .subscribe({
    next: v => addLog(`✅ Value: ${v}`, '#c8e6c9'),
    error: err => addLog(`❌ Error: ${err.message}`, '#ffcdd2'),
    complete: () => addLog('Completed', '#e3f2fd')
  });
```

- "Ignorable errors" are excluded and the stream continues
- "Critical errors" are passed to the error handler as usual
- Selective handling of errors possible

## 🧪 Practical Code Example 2: Delayed Notification

This is an example of temporarily buffering a notification and then restoring it.

```ts
import { from, interval, take, delay } from 'rxjs';
import { materialize, dematerialize, bufferTime, concatMap } from 'rxjs';

// UI creation
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'dematerialize - Buffering and delay';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('en-US', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Start - emit values every second, process in batches every 2 seconds');

interval(1000)
  .pipe(
    take(6),
    materialize(),
    bufferTime(2000),      // Buffer every 2 seconds
    concatMap(notifications => {
      addLog2(`--- Processing ${notifications.length} notifications from buffer ---`);
      return from(notifications).pipe(
        delay(500),        // Delay each notification by 0.5 seconds
        dematerialize()    // Restore to original format
      );
    })
  )
  .subscribe({
    next: v => addLog2(`Value: ${v}`),
    complete: () => addLog2('Completed')
  });
```

- Buffers notifications every 2 seconds
- Retrieve from buffer and delay processing
- Restore as original stream with `dematerialize`

## 🆚 Relationship with materialize

```ts
import { of } from 'rxjs';
import { materialize, dematerialize, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),           // Convert to Notification
    map(notification => {
      // Process as Notification object
      console.log('kind:', notification.kind);
      return notification;
    }),
    dematerialize()          // Restore
  )
  .subscribe(v => console.log('Value:', v));
// Output:
// kind: N
// Value: 1
// kind: N
// Value: 2
// kind: N
// Value: 3
// kind: C
```

| Process Flow | Description |
|:---|:---|
| Original stream | Normal value (next), error (error), completion (complete) |
| ↓ `materialize()` | Stream of Notification object |
| Intermediate processing | Processing and filtering as Notification |
| ↓ `dematerialize()` | Restore to normal stream |
| Final stream | Normal value, error, complete |

## ⚠️ Important Notes

### 1. Error Notifications Are Converted to Actual Errors

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Convert each Observable to notification object with materialize()
concat(
  of(1).pipe(materialize()),
  throwError(() => new Error('Error')).pipe(materialize()),
  of(2).pipe(materialize())  // Not executed after error
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Value:', v),
    error: err => console.log('Error:', err.message)
  });
// Output:
// Value: 1
// Error: Error
```

When an error notification is reached, the stream is interrupted with an error.

### 2. Completion Notification Completes the Stream

```ts
import { of, EMPTY, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Convert each Observable to notification object with materialize()
concat(
  of(1).pipe(materialize()),
  of(2).pipe(materialize()),
  EMPTY.pipe(materialize()),  // Completion notification
  of(3).pipe(materialize())   // Not executed after completion
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Value:', v),
    complete: () => console.log('Completed')
  });
// Output:
// Value: 1
// Value: 2
// Completed
```

No value is issued after the completion notification.

### 3. Invalid Notification Object

The `dematerialize` expects a correct Notification object.

```ts
import { of } from 'rxjs';
import { dematerialize } from 'rxjs';

// ❌ Passing normal values to dematerialize causes error
of(1, 2, 3)
  .pipe(
    dematerialize()  // Not a Notification object
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Error:', err.message)
  });
// Error occurs
```

## Practical Combination Examples

```ts
import { interval, throwError, of, concat } from 'rxjs';
import { materialize, dematerialize, take, mergeMap, map } from 'rxjs';

// Example of converting errors to warnings
interval(500)
  .pipe(
    take(10),
    mergeMap(value => {
      // Generate error only when 5
      if (value === 5) {
        return throwError(() => new Error(`Error at value ${value}`));
      }
      return of(value);
    }),
    materialize(),
    map(notification => {
      // Convert errors to warning messages
      if (notification.kind === 'E') {
        console.warn('Warning:', notification.error?.message);
        // Emit special value instead of error (generated by materialize())
        return { kind: 'N' as const, value: -1 };
      }
      return notification;
    }),
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Value:', v),
    error: err => console.error('Error:', err),  // Not called
    complete: () => console.log('Completed')
  });
// Output:
// Value: 0, 1, 2, 3, 4
// Warning: Error at value 5
// Value: -1  (instead of error)
// Value: 6, 7, 8, 9
// Completed
```

## 📚 Related Operators

- **[materialize](./materialize)** - Convert notification to Notification object
- **[catchError](/en/guide/error-handling/retry-catch)** - Error handling
- **[retry](./retry)** - Retry on error

## ✅ Summary

The `dematerialize` operator returns the Notification object to a normal notification.

- ✅ Reverse `materialize` conversion
- ✅ Restores the notification to its original format after processing
- ✅ Allows filtering and conversion of errors
- ✅ Can be used to reorder or buffer notifications
- ⚠️ Error notifications act as actual errors
- ⚠️ Completion notification completes stream
- ⚠️ Requires correct Notification object
