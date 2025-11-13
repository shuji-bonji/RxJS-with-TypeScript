---
description: TestScheduler is a powerful tool that allows you to test RxJS time-based operators using virtual time. This guide explains marble notation, how to handle Cold and Hot Observable, and how to precisely unit test time-dependent processes such as debounceTime and delay.
---

# Testing with TestScheduler

RxJS's `TestScheduler` is a powerful tool for accurately testing time-based operators. This chapter systematically explains how to test utilizing TestScheduler.

## What is TestScheduler?

Normally, Observable works in a time-dependent manner. For example, `delay()` and `debounceTime()` are operators that wait for a certain time.
Since it is inefficient to actually wait in testing, `TestScheduler` is a mechanism to test immediately using virtual time.

> [!TIP]
> TestScheduler uses "virtual time", so there is no need to wait for real time.

## Basic configuration of TestScheduler

This is the basic test configuration using TestScheduler.

```ts
import { TestScheduler } from 'rxjs/testing';
import { describe, it, beforeEach, expect } from 'vitest';

describe('TestScheduler basics', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Simple test', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b--c|');
      const expected =  '--a--b--c|';

      expectObservable(source$).toBe(expected);
    });
  });
});
```

- `cold()`: Create a Cold Observable where the stream starts independently for each subscription
- `hot()`: Create a Hot Observable where the stream is already in progress
- `expectObservable()`: verify the Observable's output with marble notation


## Cold Observable and Hot Observable

|Type|Characteristics|Use|
|:---|:---|:---|
|Cold Observable|Data flow from the beginning for each subscription|HTTP request, etc.|
|Hot Observable|Data flow has already started and is shared with subscribers|User events, WebSockets, etc.|


## Basics of marble notation

Marble notation is a method of representing the passage of time in Observable as a string.

|Symbol|Meaning|
|:---|:---|
|`-`|Time elapsed (one frame)|
|`a`, `b`, `c`|value issued|
|`|`|Completed|
|`#`|Error|
|`() `|Multiple values issued simultaneously (multiple events)|

#### Example

```
--a--b--c|    // a after 2 frames, then b, c, complete
```


## Example test with virtual time

### Testing with debounceTime

Test the debounceTime operator using virtual time.

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { debounceTime, map } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Testing with virtual time', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Test debounceTime operator', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20),
        map(x => x.toUpperCase())
      );
      const expected =    '-----------(C|)';  // ← This!

      expectObservable(result$).toBe(expected, { B: 'B', C: 'C' });
    });
  });
});
```


## Test error handling

Test the behavior when an error occurs.

```ts
import { describe, it, expect, beforeEach } from 'vitest';
import { catchError} from 'rxjs';
import { TestScheduler } from 'rxjs/testing';
import { of } from 'rxjs';

describe('Error handling test', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Catch error with catchError', () => {
    testScheduler.run(({ cold, expectObservable }) => {
      const source$ = cold('--a--#');
      const result$ = source$.pipe(
        catchError(() => of('X'))
      );

      const expected =    '--a--(X|)';

      expectObservable(result$).toBe(expected);
    });
  });
});
```


## Summary

- TestScheduler allows testing without waiting for real time
- Understand the difference between Cold/Hot Observable and use it differently
- Visualize the passage of time using marble notation
- Test even complex asynchronous streams with precision

> [!NEXT]
> Next, we will learn more advanced marble testing (customization of marble strings and combinations of multiple streams).
