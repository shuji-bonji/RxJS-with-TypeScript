---
description: bufferCount is an RxJS conversion operator that outputs an array of values for each specified number of items. It is ideal for batch processing, data aggregation by fixed count, packet splitting, and other count-based stream control, and enables type-safe array operations through TypeScript's type inference.
---

# bufferCount - Collect Values by Specified Count

The `bufferCount` operator **groups together** a specified number of emitted values and outputs them as an array.
This is useful for batch processing where you want to separate values by count.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { bufferCount } from 'rxjs';

// Emit values every 100ms
const source$ = interval(100);

source$.pipe(
  bufferCount(5)
).subscribe(buffer => {
  console.log('Values every 5:', buffer);
});

// Output:
// Values every 5: [0, 1, 2, 3, 4]
// Values every 5: [5, 6, 7, 8, 9]
// ...
```

- Outputs an array of 5 values at a time.
- It is unique in that it groups on a **count basis**, not on a time basis.

[🌐 RxJS Official Documentation - `bufferCount`](https://rxjs.dev/api/operators/bufferCount)

## 💡 Typical Usage Patterns

- Split and send data packets
- Batch saving or batch processing by a certain count
- Aggregation of input events by a certain number of occurrences

## 🧠 Practical Code Example (with UI)

This is an example of displaying a summary of keyboard keystrokes every 5 keystrokes.

```ts
import { fromEvent } from 'rxjs';
import { map, bufferCount } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Key input event stream
fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  bufferCount(5)
).subscribe(keys => {
  const message = `5 inputs: ${keys.join(', ')}`;
  console.log(message);
  output.textContent = message;
});
```

- Each time a key is pressed five times, those five keystrokes are displayed together.
- You can experience the aggregation process according to the count.
