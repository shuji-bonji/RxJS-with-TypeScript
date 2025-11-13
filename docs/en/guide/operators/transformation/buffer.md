---
description: buffer is an RxJS operator that outputs an array of accumulated values at the time another Observable emits a value, making it ideal for event-driven batch processing.
---

# buffer - Collect Values at Another Observable's Timing

The `buffer` operator accumulates the values of a source Observable **until** another Observable emits a value, and then outputs the accumulated values as an **array** at that timing.
This is useful when you want to control buffering according to external events or signals, rather than by time or number of items.

## 🔰 Basic Syntax and Usage

```ts
import { interval, fromEvent } from 'rxjs';
import { buffer } from 'rxjs';

// Emit values every 100ms
const source$ = interval(100);

// Use click event as trigger
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  buffer(clicks$)
).subscribe(bufferedValues => {
  console.log('Values accumulated until click:', bufferedValues);
});

// Output example (outputs on each click):
// Values accumulated until click: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
// Values accumulated until click: [11, 12, 13, 14, 15, 16, 17]
// ...
```

- Each time `clicks$` emits a value, the values accumulated up to that point are output as an array.
- The feature is that buffer delimitation can be controlled by an external Observable.

[🌐 RxJS Official Documentation - `buffer`](https://rxjs.dev/api/operators/buffer)

## 💡 Typical Usage Patterns

- Batch processing triggered by user actions
- Data collection and transmission based on external signals
- Event grouping with dynamic delimitation
- Batch send when WebSocket or API connection is established

## 🔍 Difference from bufferTime / bufferCount

| Operator | Timing of Delimiter | Usage |
|:---|:---|:---|
| `buffer` | **Another Observable emits** | Event-driven control |
| `bufferTime` | **Fixed time interval** | Time-based batch processing |
| `bufferCount` | **Fixed count** | Count-based batch processing |

```ts
import { interval, timer } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);
// Trigger every 1 second
const trigger$ = timer(1000, 1000);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Values every second:', values);
});

// Output:
// Values every second: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Values every second: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
```

## 🧠 Practical Code Example (with UI)

This is an example of triggering a button click and recording all mouse movement events up to that point together.

```ts
import { fromEvent } from 'rxjs';
import { map, buffer } from 'rxjs';

// Create button and output area
const button = document.createElement('button');
button.textContent = 'Record Mouse Movement';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Mouse move event
const mouseMoves$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
);

// Trigger on button click
const clicks$ = fromEvent(button, 'click');

mouseMoves$.pipe(
  buffer(clicks$)
).subscribe(positions => {
  const message = `Detected events: ${positions.length} items`;
  console.log(message);
  console.log('Coordinate data:', positions.slice(0, 5)); // Display only first 5
  output.textContent = message;
});
```

- All mouse movements up to the button click are stored in a buffer.
- Since the events are processed together at the time of the click, batch processing at arbitrary timing is possible.

## 🎯 Advanced Example with Multiple Triggers

More flexible control is possible by combining multiple trigger Observables.

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { buffer, mapTo } from 'rxjs';

const source$ = interval(100);

// Multiple triggers: click or 5 seconds elapsed
const clicks$ = fromEvent(document, 'click').pipe(mapTo('click'));
const fiveSeconds$ = timer(5000, 5000).pipe(mapTo('timer'));
const trigger$ = merge(clicks$, fiveSeconds$);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log(`Buffer output (${values.length} items):`, values);
});
```

## ⚠️ Notes

### Beware of Memory Leaks

Because `buffer` keeps accumulating values until the next trigger, it may consume excessive memory if a trigger does not occur for a long time.

```ts
// Bad example: Trigger may not occur
const neverTrigger$ = fromEvent(document.querySelector('.non-existent'), 'click');

source$.pipe(
  buffer(neverTrigger$) // Trigger never occurs, buffer accumulates infinitely
).subscribe();
```

**Countermeasures**:
- Limit the maximum buffer size in combination with `bufferTime` and `bufferCount`
- Add timeout handling

```ts
import { interval, fromEvent, timer, race } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);

// Multiple triggers: click or 5 seconds elapsed
const clicks$ = fromEvent(document, 'click');
const timeout$ = timer(10000); // Timeout after maximum 10 seconds

source$.pipe(
  buffer(race(clicks$, timeout$)) // Emit on whichever comes first
).subscribe(values => {
  console.log('Buffer:', values);
});
```

## 📚 Related Operators

- [`bufferTime`](/en/guide/operators/transformation/bufferTime) - Time-based buffering
- [`bufferCount`](/en/guide/operators/transformation/bufferCount) - Count-based buffering
- [`bufferToggle`](https://rxjs.dev/api/operators/bufferToggle) - Buffering control with start and end Observable
- [`bufferWhen`](https://rxjs.dev/api/operators/bufferWhen) - Buffering with dynamic closing conditions
- [`window`](/en/guide/operators/transformation/windowTime) - Returns Observable instead of buffer

## Summary

The `buffer` operator is a powerful tool for processing a batch of values triggered by an external Observable. It allows for **event-driven** batch processing, rather than time or number of items. However, beware of memory leaks when triggers do not occur.
