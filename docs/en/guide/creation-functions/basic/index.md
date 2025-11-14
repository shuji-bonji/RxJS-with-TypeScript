---
description: This section covers Creation Functions for basic Observable creation, using of, from, fromEvent, interval, and timer to create Observables from a variety of data sources including single values, arrays, Promises, events, and timers. This is an important concept that can be implemented with TypeScript type safety and is the foundation of RxJS.
---

# Basic Creation Functions

The most basic and frequently used Creation Functions. Easily create data, arrays, events, and time-based Observables.

## What are Basic Creation Functions?

Basic Creation Functions are functions for creating a single Observable from a variety of data sources. They are the most fundamental set of functions in RxJS and are used in almost all RxJS code.

Please review the table below to see the characteristics and usage of each Creation Function.

## Major Basic Creation Functions

| Function | Description | Use Cases |
|----------|------|-------------|
| **[of](/en/guide/creation-functions/basic/of)** | Emit specified values in sequence | Testing with fixed values, creating mocks |
| **[from](/en/guide/creation-functions/basic/from)** | Convert from array, Promise, etc. | Streaming existing data |
| **[fromEvent](/en/guide/creation-functions/basic/fromEvent)** | Convert events to Observable | DOM events, Node.js EventEmitter |
| **[interval](/en/guide/creation-functions/basic/interval)** | Emit continuously at specified intervals | Polling, periodic execution |
| **[timer](/en/guide/creation-functions/basic/timer)** | Start emitting after a delay | Delayed execution, timeout |

## Usage Criteria

The choice of Basic Creation Functions is determined by the type of data source.

### 1. Data Type

- **Static values**: `of()` - Creates an Observable by specifying the value directly
- **Array or Iterable**: `from()` - Convert an existing collection to a stream
- **Promise**: `from()` - Convert asynchronous processing to Observable
- **Event**: `fromEvent()` - Converts an event listener to an Observable
- **Time-based**: `interval()`, `timer()` - Emit values based on the passage of time

### 2. Timing of Emission

- **Emit immediately**: `of()`, `from()` - Start emitting values upon subscription
- **On event occurrence**: `fromEvent()` - Emit whenever an event occurs
- **Emit periodically**: `interval()` - Emit continuously at regular intervals
- **Emit after a delay**: `timer()` - Start emitting after a specified time

### 3. Timing of Completion

- **Complete immediately**: `of()`, `from()` - Complete after all values are emitted
- **Never complete**: `fromEvent()`, `interval()` - Continue until unsubscribe
- **Emit once and complete**: `timer(delay)` - Complete after emitting one value

## Practical Usage Examples

### of() - Testing with Fixed Values

```typescript
import { of } from 'rxjs';

// Create test data
const mockUser$ = of({ id: 1, name: 'Test User' });

mockUser$.subscribe(user => console.log(user));
// Output: { id: 1, name: 'Test User' }
```

### from() - Streaming an Array

```typescript
import { from } from 'rxjs';
import { map } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

numbers$.pipe(
  map(n => n * 2)
).subscribe(console.log);
// Output: 2, 4, 6, 8, 10
```

### fromEvent() - Click Event

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Button clicked!'));
```

### interval() - Polling

```typescript
import { interval } from 'rxjs';
import { switchMap } from 'rxjs';

// Poll API every 5 seconds
interval(5000).pipe(
  switchMap(() => fetchData())
).subscribe(data => console.log('Updated:', data));
```

### timer() - Delayed Execution

```typescript
import { timer } from 'rxjs';

// Execute after 3 seconds
timer(3000).subscribe(() => console.log('3 seconds passed'));
```

## Watch Out for Memory Leaks

Proper unsubscription is important when using the Basic Creation Functions.

> [!WARNING]
> `fromEvent()`, `interval()`, and periodic `timer(delay, period)` will not complete and must always be `unsubscribe()`d or automatically unsubscribed with `takeUntil()` or similar when the component is destroyed.
>
> Note: `timer(delay)` without a second argument will automatically complete after emitting once.

```typescript
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => console.log('Window resized'));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Converting Cold to Hot

As shown in the table above, **all Basic Creation Functions generate Cold Observables**. Each subscription initiates an independent execution.

However, you can **convert a Cold Observable to a Hot Observable** by using the following multicast operators.

### Conditions and Operators for Conversion to Hot

| Operator | Behavior | Use Cases |
|-------------|------|-------------|
| **share()** | Multicast + automatic connect/disconnect | Share HTTP requests among multiple subscribers |
| **shareReplay(n)** | Cache the latest n values and deliver to new subscribers | Cache API responses |
| **publish() + connect()** | Manually start multicast | Start execution when subscribers are ready |
| **multicast(subject)** | Multicast with custom Subject | When advanced control is needed |

### Practical Example

```typescript
import { interval } from 'rxjs';
import { take, share } from 'rxjs';

// ❄️ Cold - Independent timer for each subscription
const cold$ = interval(1000).pipe(take(3));

cold$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  cold$.subscribe(val => console.log('B:', val));
}, 1500);

// Output:
// A: 0 (after 0s)
// A: 1 (after 1s)
// B: 0 (after 1.5s) ← B starts independently from 0
// A: 2 (after 2s)
// B: 1 (after 2.5s)

// 🔥 Hot - Shares timer among subscribers
const hot$ = interval(1000).pipe(take(3), share());

hot$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  hot$.subscribe(val => console.log('B:', val));
}, 1500);

// Output:
// A: 0 (after 0s)
// A: 1 (after 1s)
// A: 2, B: 2 (after 2s) ← B joins midway, receives same value
```

> [!TIP]
> **Cases where Hot conversion is required**:
> - Want to share an HTTP request among multiple subscribers
> - Want to maintain only one WebSocket or server connection
> - Want to use the results of high-cost calculations in multiple locations
>
> For more information, see the **Subject and Multicast** chapter (Chapter 5).

## Relationship to Pipeable Operator

There is no Pipeable Operator directly corresponding to the Basic Creation Functions. They are always used as Creation Functions.

However, they are used in combination with Pipeable Operators in the following pattern:

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, switchMap } from 'rxjs';

// User input → Wait 300ms → API call
fromEvent(input, 'input').pipe(
  debounceTime(300),
  switchMap(event => fetchSuggestions(event.target.value))
).subscribe(suggestions => console.log(suggestions));
```

## Next Steps

To learn more about how each Creation Function works and practical examples, click on the links from the table above.

Also, by learning [Combination Creation Functions](/en/guide/creation-functions/combination/), [Selection/Partition Creation Functions](/en/guide/creation-functions/selection/), and [Conditional Creation Functions](/en/guide/creation-functions/conditional/), you can understand the whole picture of Creation Functions.
