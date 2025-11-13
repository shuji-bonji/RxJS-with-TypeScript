---
description: The repeat operator reruns the entire stream a specified number of times after the source Observable completes successfully. It can be used for periodic polling, repetitive animation, and other situations that require different control from retry.
---

# repeat - Repeat Stream

The `repeat` operator reruns the entire stream a specified number of times after the source Observable has **completed successfully**.
This is useful for polling processes, repeated animations, and controls that are different from retries.

## 🔰 Basic Syntax and Operation

The simplest usage is to configure a sequence of values to repeat a certain number of times.

```ts
import { of } from 'rxjs';
import { repeat } from 'rxjs';

of('A', 'B')
  .pipe(
    repeat(2) // Repeat entire sequence 2 times (output 2 times total)
  )
  .subscribe(console.log);
// Output:
// A
// B
// A
// B
```

[🌐 RxJS Official Documentation - repeat](https://rxjs.dev/api/index/function/repeat)

## 💡 Typical Usage Example

For example, it is used for simple polling processes or repeated display animations.

```ts
import { of } from 'rxjs';
import { tap, delay, repeat } from 'rxjs';

of('✅ Data retrieved successfully')
  .pipe(
    tap(() => console.log('Request started')),
    delay(1000),
    repeat(3) // Repeat 3 times
  )
  .subscribe(console.log);
// Output:
// Request started
// ✅ Data retrieved successfully
// main.ts:6 Request started
// ✅ Data retrieved successfully
// main.ts:6 Request started
// ✅ Data retrieved successfully
```

In this example, "request → data retrieval" is repeated three times every second.

## 🧪 Practical Code Example (with UI)

```ts
import { of } from 'rxjs';
import { repeat, tap } from 'rxjs';

// Output display area
const repeatOutput = document.createElement('div');
repeatOutput.innerHTML = '<h3>repeat Example:</h3>';
document.body.appendChild(repeatOutput);

// Repeat count display
let repeatCount = 0;
const repeatCountDisplay = document.createElement('div');
repeatCountDisplay.textContent = `Repeat count: ${repeatCount}`;
repeatCountDisplay.style.fontWeight = 'bold';
repeatOutput.appendChild(repeatCountDisplay);

// Values output area
const valuesOutput = document.createElement('div');
valuesOutput.style.marginTop = '10px';
valuesOutput.style.padding = '10px';
valuesOutput.style.border = '1px solid #ddd';
valuesOutput.style.maxHeight = '200px';
valuesOutput.style.overflowY = 'auto';
repeatOutput.appendChild(valuesOutput);

// Sequence repetition
of('A', 'B', 'C')
  .pipe(
    tap(() => {
      repeatCount++;
      repeatCountDisplay.textContent = `Repeat count: ${repeatCount}`;
    }),
    repeat(3)
  )
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Value: ${val} (repeat ${repeatCount})`;
    valuesOutput.appendChild(valueItem);
  });

```

## ✅ Summary

- `repeat` **reruns the entire Observable after successful completion**
- Unlike `retry`, it does **not reexecute on error**
- Can be used for repetitive animations, such as polling processes and **flashing placeholders**
