---
description: "mergeWith Pipeable Operator subscribes to multiple Observables simultaneously and merges them in parallel: Ideal for integrating multiple event sources"
titleTemplate: ':title'
---

# mergeWith - Merge Streams in Parallel

The `mergeWith` operator **simultaneously subscribes** to the original Observable and the specified other Observables,
and merges the values emitted from each in real-time.
This is the Pipeable Operator version of the Creation Function `merge`.

## 🔰 Basic Syntax and Usage

```ts
import { interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Stream 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Stream 2: ${val}`),
  take(2)
);

source1$
  .pipe(mergeWith(source2$))
  .subscribe(console.log);

// Example output:
// Stream 1: 0
// Stream 2: 0
// Stream 1: 1
// Stream 1: 2
// Stream 2: 1
```

- All Observables are subscribed to simultaneously, and values flow **in the order they are emitted**.
- There is no guarantee of order, and it **depends on the timing of each Observable's emission**.

[🌐 RxJS Official Documentation - `mergeWith`](https://rxjs.dev/api/operators/mergeWith)


## 💡 Typical Usage Patterns

- **Integrate multiple event sources**: Combine user operations and automatic updates
- **Merge parallel data fetches**: Aggregate responses from multiple APIs into a single stream
- **Merge real-time updates**: Integrate WebSocket and polling


## 🧠 Practical Code Example (with UI)

Example of integrating user click events and auto-update timer to display notifications.

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

// Create output area
const output = document.createElement('div');
output.innerHTML = '<h3>mergeWith Practical Example:</h3>';
document.body.appendChild(output);

// Create button
const button = document.createElement('button');
button.textContent = 'Manual Update';
document.body.appendChild(button);

// Click stream
const manualUpdate$ = fromEvent(button, 'click').pipe(
  map(() => '👆 Manual update executed')
);

// Auto-update timer (every 5 seconds)
const autoUpdate$ = interval(5000).pipe(
  map(val => `🔄 Auto update #${val + 1}`),
  take(3)
);

// Integrate both and display
manualUpdate$
  .pipe(mergeWith(autoUpdate$))
  .subscribe((value) => {
    const timestamp = new Date().toLocaleTimeString();
    const item = document.createElement('div');
    item.textContent = `[${timestamp}] ${value}`;
    output.appendChild(item);
  });
```

- Clicking the button immediately displays manual update,
- Auto-updates also run in parallel every 5 seconds.
- Both events are integrated in real-time.


## 🔄 Difference from Creation Function `merge`

### Basic Differences

| | `merge` (Creation Function) | `mergeWith` (Pipeable Operator) |
|:---|:---|:---|
| **Usage Location** | Used as independent function | Used within `.pipe()` chain |
| **Syntax** | `merge(obs1$, obs2$, obs3$)` | `obs1$.pipe(mergeWith(obs2$, obs3$))` |
| **First Stream** | Treats all equally | Treats as main stream |
| **Advantage** | Simple and readable | Easy to combine with other operators |

### Specific Usage Examples

**Creation Function is Recommended for Simple Merging Only**

```ts
import { merge, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(map(() => 'Click'));
const moves$ = fromEvent(document, 'mousemove').pipe(map(() => 'Mouse move'));
const keypress$ = fromEvent(document, 'keypress').pipe(map(() => 'Key press'));

// Simple and readable
merge(clicks$, moves$, keypress$).subscribe(console.log);
// Output: Display in the order any event occurs
```

**Pipeable Operator is Recommended When Adding Transformation Processing to Main Stream**

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, filter, throttleTime } from 'rxjs';

const userClicks$ = fromEvent(document, 'click');
const autoRefresh$ = interval(30000); // Every 30 seconds

// ✅ Pipeable Operator version - completed in one pipeline
userClicks$
  .pipe(
    throttleTime(1000),           // Prevent rapid clicking
    map(() => ({ source: 'user', timestamp: Date.now() })),
    mergeWith(
      autoRefresh$.pipe(
        map(() => ({ source: 'auto', timestamp: Date.now() }))
      )
    ),
    filter(event => event.timestamp > Date.now() - 60000)  // Within 1 minute only
  )
  .subscribe(event => {
    console.log(`${event.source} update: ${new Date(event.timestamp).toLocaleTimeString()}`);
  });

// ❌ Creation Function version - becomes verbose
import { merge } from 'rxjs';
merge(
  userClicks$.pipe(
    throttleTime(1000),
    map(() => ({ source: 'user', timestamp: Date.now() }))
  ),
  autoRefresh$.pipe(
    map(() => ({ source: 'auto', timestamp: Date.now() }))
  )
).pipe(
  filter(event => event.timestamp > Date.now() - 60000)
).subscribe(event => {
  console.log(`${event.source} update: ${new Date(event.timestamp).toLocaleTimeString()}`);
});
```

**When Integrating Multiple Data Sources**

```ts
import { fromEvent, timer } from 'rxjs';
import { mergeWith, map, startWith } from 'rxjs';

// Create button
const saveButton = document.createElement('button');
saveButton.textContent = 'Save';
document.body.appendChild(saveButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Main stream: User's save operation
const manualSave$ = fromEvent(saveButton, 'click').pipe(
  map(() => '💾 Manual save')
);

// ✅ Pipeable Operator version - add auto-save to main stream
manualSave$
  .pipe(
    startWith('📝 Editing started'),
    mergeWith(
      timer(10000, 10000).pipe(map(() => '⏰ Auto-save'))  // Auto-save every 10 seconds
    )
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    output.appendChild(div);
  });
```

### Summary

- **`merge`**: Optimal for simply merging multiple streams on equal footing
- **`mergeWith`**: Optimal when you want to merge other streams while transforming or processing the main stream


## ⚠️ Important Notes

### Completion Timing

The merged stream will not complete until all Observables complete.

```ts
import { of, interval, NEVER } from 'rxjs';
import { mergeWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  mergeWith(
    interval(1000).pipe(take(2)),
    // NEVER  // ← Adding this will never complete
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Complete')
});
// Output: 1 → 2 → 3 → 0 → 1 → ✅ Complete
```

### Controlling Concurrent Execution Count

By default, all streams execute concurrently, but can be controlled in combination with `mergeMap`.

```ts
import { from, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

from([1, 2, 3, 4, 5]).pipe(
  mergeMap(
    val => of(val).pipe(delay(1000)),
    2  // Execute up to 2 concurrently
  )
).subscribe(console.log);
```

### Error Handling

If an error occurs in any Observable, the entire stream terminates with an error.

```ts
import { throwError, interval } from 'rxjs';
import { mergeWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  mergeWith(
    throwError(() => new Error('Error occurred')).pipe(
      catchError(err => of('Error recovered'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Error:', err.message)
});
// Output: 0 → Error recovered → 1
```


## 📚 Related Operators

- **[merge](/pt/guide/creation-functions/combination/merge)** - Creation Function version
- **[concatWith](/pt/guide/operators/combination/concatWith)** - Pipeable version for sequential combination
- **[mergeMap](/pt/guide/operators/transformation/mergeMap)** - Map each value in parallel
