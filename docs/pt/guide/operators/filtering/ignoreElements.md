---
description: The ignoreElements operator is an RxJS filtering operator that ignores all values and passes through only completion and error notifications. It is useful when waiting for process completion.
titleTemplate: ':title'
---

# ignoreElements - Apenas Conclusão

The `ignoreElements` operator ignores **all values** emitted from the source Observable and passes **only completion and error notifications** downstream.

## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Value:', value), // Not called
  complete: () => console.log('Completed')
});
// Output: Completed
```

**Flow of operation**:
1. 1, 2, 3, 4, 5 are all ignored
2. Only completion notification is propagated downstream

[🌐 RxJS Official Documentation - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Typical Usage Patterns

- **Waiting for process completion**: When values are unnecessary and only completion is needed
- **Execute side effects only**: Execute side effects with tap and ignore values
- **Error handling**: When you want to capture only errors
- **Sequence synchronization**: Wait for completion of multiple processes

## 🧠 Practical Code Example 1: Waiting for Initialization Completion

Example of waiting for multiple initialization processes to complete.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// Create UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Application Initialization';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Function to add status log
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Initialization process 1: Database connection
const initDatabase$ = from(['Connecting to DB...', 'Checking tables...', 'DB ready']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Ignore values, notify only completion
);

// Initialization process 2: Load config file
const loadConfig$ = from(['Loading config file...', 'Parsing config...', 'Config applied']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Initialization process 3: User authentication
const authenticate$ = from(['Checking credentials...', 'Validating token...', 'Authentication complete']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Execute all initialization processes
addLog('Initialization started...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ All initialization complete! Application can start.';
    addLog('Application launch', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Initialization error: ${err.message}`;
  }
});
```

- Detailed logs for each initialization process are displayed, but values are ignored.
- A completion message is displayed when all processes complete.

## 🆚 Comparison with Similar Operators

### ignoreElements vs filter(() => false) vs take(0)

| Operator | Value Processing | Completion Notification | Use Case |
|:---|:---|:---|:---|
| `ignoreElements()` | Ignore all | Pass through | **Only completion needed** (recommended) |
| `filter(() => false)` | Filter all | Pass through | Conditional filtering (coincidentally all excluded) |
| `take(0)` | Complete immediately | Pass through | Want to complete immediately |

**Recommended**: Use `ignoreElements()` when intentionally ignoring all values. It makes code intent clear.

## 🔄 Handling Error Notifications

`ignoreElements` ignores values but **passes through error notifications**.

```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Error occurred'))
).pipe(
  ignoreElements()
);

// Success case
success$.subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('✅ Complete'),
  error: err => console.error('❌ Error:', err.message)
});
// Output: ✅ Complete

// Error case
error$.subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('✅ Complete'),
  error: err => console.error('❌ Error:', err.message)
});
// Output: ❌ Error: Error occurred
```

## ⚠️ Notes

### 1. Side Effects Are Executed

`ignoreElements` ignores values but side effects (like `tap`) are executed.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Side effect:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('Complete')
});
// Output:
// Side effect: 1
// Side effect: 2
// Side effect: 3
// Complete
```

### 2. Use with Infinite Observables

With infinite Observables, subscription continues forever as completion never comes.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Bad example: Does not complete
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Complete') // Not called
});

// ✅ Good example: Complete with take
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Complete') // Called after 5 seconds
});
```

### 3. TypeScript Type

The return value of `ignoreElements` is of type `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// Result of ignoreElements is Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value is never type, so this block is not executed
    console.log(value);
  },
  complete: () => console.log('Completion only')
});
```

## 📚 Related Operators

- **[filter](/pt/guide/operators/filtering/filter)** - Filter values based on conditions
- **[take](/pt/guide/operators/filtering/take)** - Get only first N values
- **[skip](/pt/guide/operators/filtering/skip)** - Skip first N values
- **[tap](https://rxjs.dev/api/operators/tap)** - Execute side effects (official documentation)

## Summary

The `ignoreElements` operator ignores all values and passes through only completion and error.

- ✅ Ideal when only completion notification is needed
- ✅ Side effects (tap) are executed
- ✅ Also passes through error notifications
- ✅ Intent more clear than `filter(() => false)`
- ⚠️ Does not complete with infinite Observables
- ⚠️ Return value type is `Observable<never>`
- ⚠️ Values are completely ignored but side effects are executed
