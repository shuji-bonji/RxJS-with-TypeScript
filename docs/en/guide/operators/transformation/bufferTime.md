---
description: "bufferTime operator collects values at regular time intervals for batch processing: Perfect for aggregating events, logs, and real-time data streams"
---

# bufferTime - Output Collected Values at Regular Intervals

The `bufferTime` operator outputs **an array of values** at specified time intervals.
This is useful when you want to separate the stream by a certain amount of time and treat it like a batch process.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { bufferTime } from 'rxjs';

// Emit values every 100ms
const source$ = interval(100);

source$.pipe(
  bufferTime(1000)
).subscribe(buffer => {
  console.log('Values collected in 1 second:', buffer);
});

// Output example:
// Values collected in 1 second: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Values collected in 1 second: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
// ...
```

- Values emitted in one second are grouped into an array and output in sequence.

[🌐 RxJS Official Documentation - `bufferTime`](https://rxjs.dev/api/operators/bufferTime)

## 💡 Typical Usage Patterns

- Send batches at regular intervals
- Process user operations in batches (e.g., drag operations)
- Collect data from sensors and IoT devices
- Thinning and compressing log and trace information

## 🧠 Practical Code Example (with UI)

Buffer click events for 1 second and output them together every second.

```ts
import { fromEvent } from 'rxjs';
import { bufferTime } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Click event stream
const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  bufferTime(1000)
).subscribe(clickArray => {
  const message = `Clicks in 1 second: ${clickArray.length}`;
  console.log(message);
  output.textContent = message;
});
```

- The number of clicks per second is displayed as a summary.
- The buffering process allows you to manage successive occurrences of events together.
