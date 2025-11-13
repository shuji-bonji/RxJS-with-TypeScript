---
description: Unit testing in RxJS uses synchronous, asynchronous, and time-controlled techniques to build a robust testing strategy using TestScheduler, marble testing, and mocks and stubs.
---

# Unit Tests for RxJS

Code using RxJS involves a lot of asynchronous processing and requires a different approach than traditional testing methods. This guide describes both basic and advanced techniques for effectively testing code using RxJS.

## Testing Synchronous Observable

Let's start with the simplest case: testing an Observable that completes synchronously.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Function under test
function doubleValues(input$: Observable<number>) : Observable<number>{
  return input$.pipe(
    map(x => x * 2)
  );
}

describe('Basic Observable testing', () => {
  it('Doubles values', () => {
    // Test Observable
    const source$ = of(1, 2, 3);
    const result$ = doubleValues(source$);

    // Expected result
    const expected = [2, 4, 6];
    const actual: number[] = [];

    // Execution and verification
    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
      }
    });
  });
});
```

## How to test an asynchronous Observable

For asynchronous Observable, take advantage of the testing framework's asynchronous support.

```ts
import { Observable, timer } from 'rxjs';
import { map, take } from 'rxjs';
import { describe, it, expect } from 'vitest';

// Asynchronous function under test
function getDelayedValues(): Observable<number> {
  return timer(0, 100).pipe(
    map(x => x + 1),
    take(3)
  );
}

describe('Testing asynchronous Observable', () => {
  it('Receives asynchronous values in order', (done: Function) => {
    const result$ = getDelayedValues();
    const expected = [1, 2, 3];
    const actual: number[] = [];

    result$.subscribe({
      next: (value) => actual.push(value),
      complete: () => {
        expect(actual).toEqual(expected);
        done();
      }
    });
  });
});
```

## Asynchronous testing with Promise transformation

Another method is to convert an Observable to a Promise using `firstValueFrom()` or `lastValueFrom()` and utilize async/await of modern JS/TS.

```ts
import { Observable, of } from 'rxjs';
import { map, delay, toArray } from 'rxjs';
import { describe, it, expect } from 'vitest';
import { lastValueFrom } from 'rxjs';

// Function under test
function processWithDelay(input$: Observable<number>) {
  return input$.pipe(
    map(x => x * 10),
    delay(100),
    toArray()
  );
}

describe('Testing with Promise conversion', () => {
  it('Wait for delay processing before validation', async () => {
    const source$ = of(1, 2, 3);
    const result$ = processWithDelay(source$);

    // Convert Observable to promise
    const result = await lastValueFrom(result$);

    // Expected result
    expect(result).toEqual([10, 20, 30]);
  });
});
```

## Utilizing TestScheduler

RxJS provides a special scheduler called `TestScheduler` that can be used to efficiently test time-based operators.

```ts
import { TestScheduler } from 'rxjs/testing';
import { map, debounceTime } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Using TestScheduler', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Testing debounceTime', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('a--b--c--d|', { a: 1, b: 2, c: 3, d: 4 });
      const result = source.pipe(
        debounceTime(20),
        map(x => x * 10)
      );

      const expected = '----------(d|)';

      expectObservable(result).toBe(expected, { d: 40 });
    });
  });
});
```

> [!NOTE]
> Marble Test Notation
> When using `TestScheduler`, use marble diagrams to represent the passage of time.

## Make time manipulatable

When testing time-dependent code (delay, debounceTime, etc.), use the `TestScheduler` to control time.

```ts
import { TestScheduler } from 'rxjs/testing';
import { interval } from 'rxjs';
import { take, map } from 'rxjs';
import { describe, it, beforeEach } from 'vitest';

describe('Time control', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Fast-forward time for testing', () => {
    testScheduler.run(({ expectObservable }) => {
      const source = interval(1000).pipe(
        take(3),
        map(x => x + 1)
      );

      // Actually takes 3 seconds, but executes immediately in test environment
      const expected = '1s a 999ms b 999ms (c|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source).toBe(expected, values);
    });
  });
});
```

## Test error handling (TestScheduler version)

It is also important to test Observable's behavior when an error occurs.

```ts
import { TestScheduler } from 'rxjs/testing';
import { throwError, of } from 'rxjs';
import { catchError } from 'rxjs';

describe('Error handling test', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('When Observable notifies an error', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const expected =     '--a--b--#';

      expectObservable(source).toBe(expected);
    });
  });

  it('When catchError captures error and replaces it with a value', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source = cold('  --a--b--#');
      const handled = source.pipe(
        catchError(() => of('X'))
      );

      const expected =     '--a--b--(X|)';

      expectObservable(handled).toBe(expected);
    });
  });
});
```

## Marble test

For testing complex streams, use a marble diagram to intuitively represent test expectations.

### Hot Observable vs. Cold Observable

TestScheduler allows for the creation of two types of Observables: hot and cold. It is important to understand this difference when testing.

```ts
import { TestScheduler } from 'rxjs/testing';
import { Subject } from 'rxjs';
import { describe, it, beforeEach, expect } from 'vitest';

describe('Hot vs Cold Observable test', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Cold Observable creates independent streams for each subscription', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      // Cold Observable (independent for each subscriber)
      const source = cold('--a--b--c|', { a: 1, b: 2, c: 3 });

      // First subscription
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Second subscription (starts from the beginning)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Hot Observable shares streams among subscribers', () => {
    testScheduler.run(({ hot, expectObservable }) => {
      // Hot Observable (shared among subscribers)
      const source = hot('--a--b--c|', { a: 1, b: 2, c: 3 });

      // Subscribe late (receives only values after subscription starts)
      expectObservable(source, '----^').toBe('-----b--c|', { b: 2, c: 3 });

      // Subscribe from the beginning (receives all values)
      expectObservable(source).toBe('--a--b--c|', { a: 1, b: 2, c: 3 });
    });
  });

  it('Testing Hot Observable using actual Subject', () => {
    // Non-TestScheduler version
    const subject = new Subject<number>();
    const values1: number[] = [];
    const values2: number[] = [];

    // First subscriber
    const subscription1 = subject.subscribe(val => values1.push(val));

    // Emit values
    subject.next(1);
    subject.next(2);

    // Second subscriber (joins midway)
    const subscription2 = subject.subscribe(val => values2.push(val));

    // Emit more values
    subject.next(3);
    subject.complete();

    // Verification
    expect(values1).toEqual([1, 2, 3]);
    expect(values2).toEqual([3]); // Only values after subscription started

    // Cleanup
    subscription1.unsubscribe();
    subscription2.unsubscribe();
  });
});
```

> [!NOTE]
> Cold Observable generates data independently every time you subscribe, but Hot Observable shares and distributes data.

## Mocking and Stubbing

### Mocking Dependent Services

When testing services using RxJS, it is common to mock external dependencies.

```ts
import { Observable, of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
}

// Service under test
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getUsers(): Observable<User[]> {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('Service testing', () => {
  it('Filter only active users', () => {
    // Mock API service
    const mockApiService = {
      fetchUsers: vi.fn().mockReturnValue(of([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ]))
    };

    const userService = new UserService(mockApiService);
    const result$ = userService.getUsers();

    // Verification
    result$.subscribe(users => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
      expect(mockApiService.fetchUsers).toHaveBeenCalledTimes(1);
    });
  });
});
```

### Stubs

Stubs are simple objects that mimic external data or APIs on which the code under test depends.
They eliminate dependencies on external resources and allow tests to run independently.
They simply return fixed values and have no internal logic.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';
import { describe, it, expect } from 'vitest';

type User = {
  id: number;
  name: string;
  active: boolean;
};

// Service under test
class UserService {
  constructor(private apiService: { fetchUsers: Function }) {}

  getActiveUsers() {
    return this.apiService.fetchUsers().pipe(
      map((users: User[]) => users.filter(user => user.active))
    );
  }
}

describe('UserService testing', () => {
  it('Returns only active users', () => {
    // 🔹 Creating stubs
    const stubApiService = {
      fetchUsers: () => of<User[]>([
        { id: 1, name: 'Tanaka', active: true },
        { id: 2, name: 'Sato', active: false },
        { id: 3, name: 'Yamada', active: true }
      ])
    };

    // Service under test
    const userService = new UserService(stubApiService);

    // Check result
    userService.getActiveUsers().subscribe((users: User[]) => {
      expect(users.length).toBe(2);
      expect(users[0].name).toBe('Tanaka');
      expect(users[1].name).toBe('Yamada');
    });
  });
});
```

## Spy on subscriptions

Spy can be used to verify that subscriptions are being done correctly.

```ts
import { Subject } from 'rxjs';
import { describe, it, expect, vi } from 'vitest';

describe('Subscription testing', () => {
  it('Subscribing with proper handlers', () => {
    const subject = new Subject();

    // Create handler spies
    const nextSpy = vi.fn();
    const errorSpy = vi.fn();
    const completeSpy = vi.fn();

    // Subscribe
    subject.subscribe({
      next: nextSpy,
      error: errorSpy,
      complete: completeSpy
    });

    // Emit values
    subject.next('value1');
    subject.next('value2');
    subject.complete();

    // Verification
    expect(nextSpy).toHaveBeenCalledTimes(2);
    expect(nextSpy).toHaveBeenCalledWith('value1');
    expect(nextSpy).toHaveBeenCalledWith('value2');
    expect(errorSpy).not.toHaveBeenCalled();
    expect(completeSpy).toHaveBeenCalledTimes(1);
  });
});
```

## Best Practices

|Best practices|Explanation|
|---|---|
|Observe the principle of single responsibility| To write testable code, each function or class should have a single responsibility. This way, testing is simplified. |
|Mock external dependencies|External dependencies such as http requests and timers should be mocked and tested in a predictable environment. |
|Use appropriate techniques for asynchronous code| Choose appropriate methods for asynchronous testing, such as TestScheduler, done() callbacks, or async/await. |
|Utilize marble testing| For testing complex streams, use marble diagrams to represent test expectations in an intuitive manner.|

## Summary

Testing RxJS code has aspects that differ from traditional JavaScript code, such as its synchronous/asynchronous nature and time-dependent behavior. By choosing an appropriate testing methodology, you can develop high-quality reactive code with confidence. In particular, keep the following points in mind

- Simple subscription tests for synchronous Observable
- TestScheduler or Promise transformations for asynchronous processing
- Marble test for time-dependent code
- Mock external dependencies to create an independent test environment.
- Design testable code according to the principle of single responsibility

## 🔗 Related Sections

- **[Common Mistakes and Solutions](/en/guide/anti-patterns/common-mistakes#15-lack-of-testing)** - Check anti-patterns related to testing
- **[Utilizing TestScheduler](/en/guide/testing/test-scheduler)** - More detailed usage of TestScheduler
- **[Marble Testing](/en/guide/testing/marble-testing)** - Advanced marble testing techniques
