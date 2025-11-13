---
description: The timestamp operator adds a timestamp to each value and records the time the value was emitted, which can be used for performance measurement and debugging.
---

# timestamp - Add Timestamp

The `timestamp` operator **appends a timestamp** to each value in the stream. This can be used for performance measurement, debugging, and time-series analysis of events by recording the exact time the value was emitted.

## 🔰 Basic Syntax and Operation

Converts each value to a timestamped object.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

interval(1000)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(console.log);

// Output:
// { value: 0, timestamp: 1640000000000 }
// { value: 1, timestamp: 1640000001000 }
// { value: 2, timestamp: 1640000002000 }
```

The returned object has the following structure:
- `value`: The original value
- `timestamp`: Timestamp (Unix time in milliseconds)

[🌐 RxJS Official Documentation - timestamp](https://rxjs.dev/api/index/function/timestamp)

## 💡 Typical Usage Examples

- **Performance measurement**: Measure processing time
- **Event timing analysis**: Measure intervals between user actions
- **Debugging and logging**: Recording the timing of value emission
- **Time-series data recording**: Time-stamped storage of sensor data, etc.

## 🧪 Practical Code Example 1: Measuring Click Intervals

This is an example of measuring the user click interval.

```ts
import { fromEvent } from 'rxjs';
import { timestamp, pairwise, map } from 'rxjs';

// UI creation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'timestamp - Click interval measurement';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Please click';
button.style.marginBottom = '10px';
button.style.padding = '10px 20px';
button.style.fontSize = '16px';
container.appendChild(button);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '250px';
output.style.overflow = 'auto';
container.appendChild(output);

let clickCount = 0;

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.insertBefore(logItem, output.firstChild);  // Display newest at top
}

fromEvent(button, 'click')
  .pipe(
    timestamp(),
    pairwise(),
    map(([prev, curr]) => {
      const interval = curr.timestamp - prev.timestamp;
      return {
        clickNumber: clickCount + 1,
        interval: interval,
        timestamp: new Date(curr.timestamp).toLocaleTimeString('en-US')
      };
    })
  )
  .subscribe(data => {
    clickCount++;
    const color = data.interval < 500 ? '#ffcdd2' :
                  data.interval < 1000 ? '#fff9c4' : '#c8e6c9';

    const speed = data.interval < 500 ? 'Fast click!' :
                  data.interval < 1000 ? 'Normal' : 'Slow';

    addLog(
      `${data.clickNumber}th click: ${data.interval}ms interval [${speed}] (${data.timestamp})`,
      color
    );
  });

addLog('Please click the button (interval measured from 2nd click)', '#e3f2fd');
```

- Accurately measure click interval
- Color-coded display according to speed
- Records time of occurrence with timestamp

## 🧪 Practical Code Example 2: Measuring Processing Time

This is an example of measuring the time taken for each process.

```ts
import { interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// UI creation
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'timestamp - Processing time measurement';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.fontSize = '12px';
  logItem.style.fontFamily = 'monospace';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

addLog2('Processing started...');

interval(500)
  .pipe(
    take(5),
    timestamp(),  // Timestamp before processing
    map(data => {
      const start = data.timestamp;

      // Simulate heavy processing (random processing time)
      const iterations = Math.floor(Math.random() * 5000000) + 1000000;
      let sum = 0;
      for (let i = 0; i < iterations; i++) {
        sum += i;
      }

      const end = Date.now();
      const duration = end - start;

      return {
        value: data.value,
        startTime: new Date(start).toLocaleTimeString('en-US', { hour12: false }) +
                   '.' + (start % 1000).toString().padStart(3, '0'),
        duration: duration
      };
    })
  )
  .subscribe({
    next: result => {
      addLog2(
        `Value${result.value}: start=${result.startTime}, processing time=${result.duration}ms`
      );
    },
    complete: () => {
      addLog2('--- All processing complete ---');
    }
  });
```

- Record the start time of each value
- Measure the time taken for processing
- Use for performance analysis

## Using Timestamps

```ts
import { of } from 'rxjs';
import { timestamp, map } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    timestamp(),
    map(data => {
      // Processing using timestamp
      const date = new Date(data.timestamp);
      return {
        value: data.value,
        time: date.toISOString(),
        unixTime: data.timestamp
      };
    })
  )
  .subscribe(console.log);
// Output:
// { value: 'A', time: '2024-01-01T00:00:00.000Z', unixTime: 1704067200000 }
// ...
```

## ⚠️ Important Notes

### 1. Timestamp Precision

Because JavaScript's `Date.now()` is used, the precision is in milliseconds.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

// High-frequency events (1ms interval)
interval(1)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(data => {
    console.log(`Value: ${data.value}, Timestamp: ${data.timestamp}`);
  });
// May have same timestamp
```

If you need higher precision, consider using `performance.now()`.

### 2. Timestamp is at Emission Time

The timestamp is the time when the value was emitted, not when it was generated.

```ts
import { of, delay, timestamp } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delay(1000),      // 1 second delay
    timestamp()       // Timestamp after delay
  )
  .subscribe(console.log);
```

### 3. Object Structure Change

Using `timestamp` wraps the value into an object.

```ts
import { of, timestamp, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    timestamp(),
    map(data => data.value * 2)  // Access original value with .value
  )
  .subscribe(console.log);
// Output: 2, 4, 6
```

## 📚 Related Operators

- **[tap](./tap)** - Perform side effects (for debugging)
- **[delay](./delay)** - Fixed time delay
- **[timeout](./timeout)** - Timeout control

## ✅ Summary

The `timestamp` operator gives a timestamp for each value.

- ✅ Accurately records the time each value is emitted
- ✅ Useful for performance measurement
- ✅ Allows analysis of event intervals
- ✅ Useful for debugging and logging
- ⚠️ Precision in milliseconds
- ⚠️ Values are wrapped in objects
- ⚠️ Timestamps are at time of emission
