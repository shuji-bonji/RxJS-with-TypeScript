---
description: This page details 15 common anti-patterns when using RxJS with TypeScript and their respective solutions. Learn practical anti-patterns such as external publication of Subject, nested subscribe, memory leaks, and misuse of shareReplay.
---

# Common Mistakes and How to Deal with Them

This page details 15 common anti-patterns when using RxJS with TypeScript and their respective solutions.

## Table of Contents

1. [External publication of Subject](#1-external-publication-of-subject)
2. [Nested subscribe (callback hell)](#2-nested-subscribe-callback-hell)
3. [unsubscribe forgetting (memory leak)](#3-unsubscribe-forgetting-memory-leak)
4. [Misuse of shareReplay](#4-misuse-of-sharereplay)
5. [Side effects in map](#5-side-effects-in-map)
6. [Ignoring Cold/Hot Observable differences](#6-ignoring-cold-hot-observable-differences)
7. [Improper mixing of Promise and Observable](#7-improper-mixing-of-promise-and-observable)
8. [Ignoring backpressure](#8-ignoring-backpressure)
9. [Error suppression](#9-error-suppression)
10. [DOM event subscription leaks](#10-dom-event-subscription-leaks)
11. [Lack of type safety (excessive use of any)](#11-lack-of-type-safety-excessive-use-of-any)
12. [Improper operator selection](#12-improper-operator-selection)
13. [Overcomplication](#13-overcomplication)
14. [State changes in subscribe](#14-state-changes-in-subscribe)
15. [Lack of testing](#15-lack-of-testing)


## 1. External publication of Subject

### Issue

If Subject is exposed as is, `next()` will be called from the outside and state management will be unpredictable.

### ❌ Bad example

```ts
import { Subject } from 'rxjs';

// Export Subject as is
export const cartChanged$ = new Subject<void>();

// Anyone can call next() from another file
cartChanged$.next(); // May be called at unexpected timing
```

### ✅ Good example

```ts
import { BehaviorSubject, Observable } from 'rxjs';

class CartStore {
  private readonly _items$ = new BehaviorSubject<string[]>([]);

  // Publish as read-only Observable
  readonly items$: Observable<string[]> = this._items$.asObservable();

  // State changes are controlled by dedicated methods
  add(item: string): void {
    this._items$.next([...this._items$.value, item]);
  }

  remove(item: string): void {
    this._items$.next(
      this._items$.value.filter(i => i !== item)
    );
  }
}

export const cartStore = new CartStore();
```

### Explanation

- Convert to read-only Observable with `asObservable()`
- Allow state changes only via dedicated methods
- Improves traceability of changes and facilitates debugging


## 2. Nested subscribe (callback hell)

### Issue

Calling more `subscribe` in a `subscribe` causes a callback hell, which complicates error handling and cancellation processing.

### ❌ Bad example

```ts
import { of } from 'rxjs';

// API call simulation
function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
}

// Nested subscribe
apiA().subscribe(a => {
  apiB(a.id).subscribe(b => {
    apiC(b.token).subscribe(result => {
      console.log('done', result);
    });
  });
});
```

### ✅ Good example

```ts
import { of } from 'rxjs';
import { switchMap } from 'rxjs';

function apiA() {
  return of({ id: 1 });
}

function apiB(id: number) {
  return of({ id, token: 'abc123' });
}

function apiC(token: string) {
  return of({ success: true });
};


// Flatten using higher-order operators
apiA().pipe(
  switchMap(a => apiB(a.id)),
  switchMap(b => apiC(b.token))
).subscribe(result => {
  console.log('done', result);
});
```

### Explanation

- Use higher-order operators such as `switchMap`, `mergeMap`, and `concatMap`
- Error handling can be done in one place
- Only one time to unsubscribe
- Improved code readability


## 3. unsubscribe forgetting (memory leak)

### Issue

Failure to unsubscribe an infinite stream (e.g., an event listener) causes a memory leak.

### ❌ Bad example

```ts
import { fromEvent } from 'rxjs';

// During component initialization
function setupResizeHandler() {
  fromEvent(window, 'resize').subscribe(() => {
    console.log('resized');
  });
  // Not unsubscribed!
}

// Event listener remains even after component is destroyed
```

### ✅ Good example

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil, finalize } from 'rxjs';

class MyComponent {
  private readonly destroy$ = new Subject<void>();

  ngOnInit(): void {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$),
      finalize(() => console.log('cleanup'))
    ).subscribe(() => {
      console.log('resized');
    });
  }

  ngOnDestroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### ✅ Another good example (how to use Subscription)

```ts
import { fromEvent, Subscription } from 'rxjs';

class MyComponent {
  private subscription = new Subscription();

  ngOnInit(): void {
    this.subscription.add(
      fromEvent(window, 'resize').subscribe(() => {
        console.log('resized');
      })
    );
  }

  ngOnDestroy(): void {
    this.subscription.unsubscribe();
  }
}
```

### Explanation

- The `takeUntil` pattern is recommended (declarative and unambiguous)
- Manual management with Subscription is also effective
- Always unsubscribe when destroying components


## 4. Misuse of shareReplay

### Issue

Using `shareReplay` without understanding how it works can result in replay of old data and memory leaks.

### ❌ Bad example

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Making buffer size unlimited
const shared$ = interval(1000).pipe(
  shareReplay() // Default is unlimited buffer
);

// Values remain in memory even when there are no subscribers
```

### ✅ Good example

```ts
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

// Explicitly specify buffer size and reference count
const shared$ = interval(1000).pipe(
  take(10),
  shareReplay({
    bufferSize: 1,
    refCount: true // Release resources when there are no more subscribers
  })
);
```

### Explanation

- Explicitly specify `bufferSize` (usually 1)
- `refCount: true` for automatic release when there are no more subscribers
- `shareReplay({ bufferSize: 1, refCount: true })` is safe for streams that complete, such as HTTP requests


## 5. Side effects in map

### Issue

Changing state in the `map` operator causes unpredictable behavior.

### ❌ Bad example

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

let counter = 0;

const source$ = of(1, 2, 3).pipe(
  map(value => {
    counter++; // Side effect!
    return value * 2;
  })
);

source$.subscribe(console.log);
source$.subscribe(console.log); // counter unexpectedly increases
```

### ✅ Good example

```ts
import { of } from 'rxjs';
import { map, tap, scan } from 'rxjs';

// Pure transformation only
const source$ = of(1, 2, 3).pipe(
  map(value => value * 2)
);

// Separate side effects with tap
const withLogging$ = source$.pipe(
  tap(value => console.log('Processing:', value))
);

// Use scan to accumulate state
const withCounter$ = of(1, 2, 3).pipe(
  scan((acc, value) => ({ count: acc.count + 1, value }), { count: 0, value: 0 })
);
```

### Explanation

- `map` is used as a pure function
- Side effects (logs, API calls, etc.) are separated into `tap`
- Use `scan` or `reduce` to accumulate state


## 6. Ignoring Cold/Hot Observable differences

### Issue

Using an Observable without understanding whether it is Cold or Hot can lead to duplicate executions and unexpected behavior.

### ❌ Bad example

```ts
import { ajax } from 'rxjs/ajax';

// Cold Observable - HTTP request is executed per subscription
const data$ = ajax.getJSON('https://api.example.com/data');

data$.subscribe(console.log); // Request 1
data$.subscribe(console.log); // Request 2 (wasteful duplication)
```

### ✅ Good example

```ts
import { ajax } from 'rxjs/ajax';
import { shareReplay } from 'rxjs';

// Convert to Hot Observable and share
const data$ = ajax.getJSON('https://api.example.com/data').pipe(
  shareReplay({ bufferSize: 1, refCount: true })
);

data$.subscribe(console.log); // Request 1
data$.subscribe(console.log); // Use cached result
```

### Explanation

- Cold Observable: executed per subscription (`of`, `from`, `fromEvent`, `ajax`, etc.)
- Hot Observable: executed regardless of subscription (Subject, multicast Observable, etc.)
- Cold can be converted to Hot with `share` / `shareReplay`


## 7. Improper mixing of Promise and Observable

### Issue

Mixing Promise and Observable without proper conversion leads to incomplete error handling and cancellation handling.

### ❌ Bad example

```ts
import { from } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Using Promise as is
from(fetchData()).subscribe(data => {
  fetchData().then(moreData => { // Nested Promise
    console.log(data, moreData);
  });
});
```

### ✅ Good example

```ts
import { from } from 'rxjs';
import { switchMap } from 'rxjs';

async function fetchData(): Promise<string> {
  return 'data';
}

// Convert Promise to Observable and unify
from(fetchData()).pipe(
  switchMap(() => from(fetchData()))
).subscribe(moreData => {
  console.log(moreData);
});
```

### Explanation

- Convert Promise to Observable with `from`
- Uniform processing in Observable pipeline
- Easier error handling and cancellation


## 8. Ignoring backpressure

### Issue

Uncontrolled handling of high frequency events results in poor performance.

### ❌ Bad example

```ts
import { fromEvent } from 'rxjs';

// Process input events as is
fromEvent(document.getElementById('search'), 'input').subscribe(event => {
  // API call on every input (overload)
  searchAPI((event.target as HTMLInputElement).value);
});

function searchAPI(query: string): void {
  console.log('Searching for:', query);
}
```

### ✅ Good example

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, distinctUntilChanged, map, switchMap } from 'rxjs';

// Apply debounce and cancellation
fromEvent(document.getElementById('search'), 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(300), // Wait 300ms
  distinctUntilChanged(), // Only when value changes
  switchMap(query => searchAPI(query)) // Cancel old requests
).subscribe(results => {
  console.log('Results:', results);
});
```

### Explanation

- `debounceTime` to wait for a certain period of time
- `throttleTime` limits maximum frequency
- `distinctUntilChanged` to exclude duplicates
- Cancel old requests with `switchMap`


## 9. Error suppression

### Issue

Failure to properly handle errors makes debugging difficult and degrades the user experience.

### ❌ Bad example

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

// Ignoring error
ajax.getJSON('https://api.example.com/data').pipe(
  catchError(() => of(null)) // Error information is lost
).subscribe(data => {
  console.log(data); // Cause unknown even if null comes
});
```

### ✅ Good example

```ts
import { ajax } from 'rxjs/ajax';
import { catchError } from 'rxjs';
import { of } from 'rxjs';

interface ApiResponse {
  data: unknown;
  error?: string;
}

ajax.getJSON<ApiResponse>('https://api.example.com/data').pipe(
  catchError(error => {
    console.error('API Error:', error);
    // Notify user
    showErrorToast('Failed to retrieve data');
    // Return alternative value with error information
    return of({ data: null, error: error.message } as ApiResponse);
  })
).subscribe((response) => {
  if (response.error) {
    console.log('Fallback mode due to:', response.error);
  }
});

function showErrorToast(message: string): void {
  console.log('Toast:', message);
}
```

### Explanation

- Logs errors
- Provides feedback to the user
- Return alternate values with error information
- Consider retry strategies (`retry`, `retryWhen`)


## 10. DOM event subscription leaks

### Issue

Failure to properly release DOM event listeners results in memory leaks.

### ❌ Bad example

```ts
import { fromEvent } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;

  constructor() {
    this.button = document.createElement('button');

    // Register event listener
    fromEvent(this.button, 'click').subscribe(() => {
      console.log('clicked');
    });

    // Not unsubscribed
  }

  destroy(): void {
    this.button.remove();
    // Listener remains
  }
}
```

### ✅ Good example

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class Widget {
  private button: HTMLButtonElement;
  private readonly destroy$ = new Subject<void>();

  constructor() {
    this.button = document.createElement('button');

    fromEvent(this.button, 'click').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => {
      console.log('clicked');
    });
  }

  destroy(): void {
    this.destroy$.next();
    this.destroy$.complete();
    this.button.remove();
  }
}
```

### Explanation

- Unsubscribe reliably with the `takeUntil` pattern
- Fires `destroy$` when component is destroyed
- Release listeners before deleting DOM elements


## 11. Lack of type safety (excessive use of any)

### Issue

Heavy use of `any` disables TypeScript type checking and is prone to runtime errors.

### ❌ Bad example

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

function fetchUser(): Observable<any> {
  return new Observable(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// Type checking doesn't work
fetchUser().pipe(
  map(user => user.naem) // Typo! Won't notice until runtime
).subscribe(console.log);
```

### ✅ Good example

```ts
import { Observable } from 'rxjs';
import { map } from 'rxjs';

interface User {
  name: string;
  age: number;
}

function fetchUser(): Observable<User> {
  return new Observable<User>(subscriber => {
    subscriber.next({ name: 'John', age: 30 });
  });
}

// Type checking works
fetchUser().pipe(
  map(user => user.name) // Error detection at compile time
).subscribe(console.log);
```

### Explanation

- Define interfaces and type aliases
- Explicit type parameters for `Observable<T>`
- Make the most of TypeScript's type inference


## 12. Improper operator selection

### Issue

Using an operator that is not fit for purpose leads to inefficient or unexpected behavior.

### ❌ Bad example

```ts
import { fromEvent } from 'rxjs';
import { mergeMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Search on every button click (old requests are not canceled)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  mergeMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### ✅ Good example

```ts
import { fromEvent } from 'rxjs';
import { switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Process only the latest request (old requests are automatically canceled)
fromEvent(document.getElementById('search-btn'), 'click').pipe(
  switchMap(() => ajax.getJSON('https://api.example.com/search'))
).subscribe(console.log);
```

### Distinguishing between major higher-order operators

| Operator | Use |
|---|---|
| `switchMap` | Process only the latest stream (search, autocomplete) |
| `mergeMap` | Concurrent processing (any order) |
| `concatMap` | Sequential processing (order is important) |
| `exhaustMap` | Ignore new input during execution (prevent consecutive button presses) |

### Explanation

- Understanding the behavior of each operator
- Select the right one for your use case
- See [Transformation Operators](/en/guide/operators/transformation/) for details


## 13. Overcomplication

### Issue

A case in which RxJS overcomplicates a process that could be written simply.

### ❌ Bad example

```ts
import { Observable, of } from 'rxjs';
import { map, mergeMap, toArray } from 'rxjs';

// Complicating simple array transformation with RxJS
function doubleNumbers(numbers: number[]): Observable<number[]> {
  return of(numbers).pipe(
    mergeMap(arr => of(...arr)),
    map(n => n * 2),
    toArray()
  );
}
```

### ✅ Good example

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Regular JavaScript is enough for array processing
function doubleNumbers(numbers: number[]): number[] {
  return numbers.map(n => n * 2);
}

// Use RxJS for asynchronous and event-driven processing
const button = document.getElementById('calc-btn') as HTMLButtonElement;
const numbers = [1, 2, 3, 4, 5];

fromEvent(button, 'click').pipe(
  map(() => doubleNumbers(numbers))
).subscribe(result => console.log(result));
```

### Explanation

- RxJS is used for asynchronous processing and event streams
- Regular JavaScript is sufficient for synchronous array processing
- Consider the balance between complexity and benefits


## 14. State changes in subscribe

### Issue

Changing state directly within `subscribe` is difficult to test and causes bugs.

### ❌ Bad example

```ts
import { interval } from 'rxjs';

class Counter {
  count = 0;

  start(): void {
    interval(1000).subscribe(() => {
      this.count++; // State change within subscribe
      this.updateUI();
    });
  }

  updateUI(): void {
    console.log('Count:', this.count);
  }
}
```

### ✅ Good example

```ts
import { interval, BehaviorSubject } from 'rxjs';
import { scan, tap } from 'rxjs';

class Counter {
  private readonly count$ = new BehaviorSubject<number>(0);

  start(): void {
    interval(1000).pipe(
      scan(acc => acc + 1, 0),
      tap(count => this.count$.next(count))
    ).subscribe();

    // UI subscribes to count$
    this.count$.subscribe(count => this.updateUI(count));
  }

  updateUI(count: number): void {
    console.log('Count:', count);
  }
}
```

### Explanation

- State is managed by `BehaviorSubject` and `scan`
- `subscribe` is used as a trigger
- Testable and reactive design


## 15. Lack of testing

### Issue

Deploying RxJS code to production without testing is prone to regressions.

### ❌ Bad example

```ts
import { interval } from 'rxjs';
import { map, filter } from 'rxjs';

// Deploy without testing
export function getEvenNumbers() {
  return interval(1000).pipe(
    filter(n => n % 2 === 0),
    map(n => n * 2)
  );
}
```

### ✅ Good example

```ts
import { TestScheduler } from 'rxjs/testing';
import { getEvenNumbers } from './numbers';

describe('getEvenNumbers', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('should emit only even numbers doubled', () => {
    scheduler.run(({ expectObservable }) => {
      const expected = '1s 0 1s 4 1s 8';
      expectObservable(getEvenNumbers()).toBe(expected);
    });
  });
});
```

### Explanation

- Marble Testing with TestScheduler
- Asynchronous processing can be tested synchronously
- See [Testing Techniques](/en/guide/testing/unit-tests) for details


## Summary

By understanding and avoiding these 15 anti-patterns, you can write more robust and maintainable RxJS code.

## References

This collection of anti-patterns has been prepared with reference to the following reliable sources.

### Official Documentation Repository
- **[RxJS Official Documentation](https://rxjs.dev/)** - Official operator and API reference
- **[GitHub Issue #5931](https://github.com/ReactiveX/rxjs/issues/5931)** - Discussion of shareReplay memory leak issue

### Anti-patterns and best practices
- **[RxJS in Angular - Antipattern 1: Nested subscriptions](https://www.thinktecture.com/en/angular/rxjs-antipattern-1-nested-subs/)** - Thinktecture AG
- **[RxJS in Angular - Antipattern 2: Stateful Streams](https://www.thinktecture.com/en/angular/rxjs-antipattern-2-state/)** - Thinktecture AG
- **[RxJS Best Practices in Angular 16 (2025)](https://www.infoq.com/articles/rxjs-angular16-best-practices/)** - InfoQ (May 2025)
- **[RxJS: Why memory leaks occur when using a Subject](https://angularindepth.com/posts/1433/rxjs-why-memory-leaks-occur-when-using-a-subject)** - Angular In Depth
- **[RxJS Antipatterns](https://brianflove.com/posts/2017-11-01-ngrx-anti-patterns/)** - Brian Love

### Additional Resources
- **[Learn RxJS](https://www.learnrxjs.io/)** - Practical guide to operators and patterns
- **[RxJS Marbles](https://rxmarbles.com/)** - Visual understanding of operators

## Utilized for code review

Check your code for anti-patterns.

👉 **[Anti-pattern Avoidance Checklist](./checklist)** - Review your code with 15 items to check

From each check item, you can jump directly to the corresponding anti-pattern details on this page.

## Next Steps

- **[Error Handling](/en/guide/error-handling/strategies)** - Learn more detailed error handling strategies
- **[Testing Techniques](/en/guide/testing/unit-tests)** - Learn how to effectively test RxJS code
- **[Understanding Operators](/en/guide/operators/)** - Learn how to use each operator in detail

Incorporate these best practices into your daily coding to write quality RxJS code!
