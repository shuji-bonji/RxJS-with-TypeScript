---
description: Marble testing is a technique that allows you to test RxJS asynchronous streams with a visual representation using strings. This guide carefully explains the difference between Cold and Hot Observable, the rules of marble notation, and how to use TestScheduler from basic to practical examples.
---

# Introduction to Marble Testing

RxJS provides a technique called "marble testing" that allows you to test the behavior of asynchronous streams **with a visual representation**.

In this section, we will learn the basics of marble testing through simple examples.

## What is Marble Notation?

Marble notation is a way to represent **time passage and event occurrence** with strings.

### Basic rules

These symbols are used to represent the passage of time and the occurrence of events.

| Symbol | Meaning |
|:----|:----|
| `-` | Time passes (advance 1 frame) |
| `a`, `b`, `c` | Emitted values (arbitrary characters) |
| `|` | Complete |
| `#` | Error |

For example.

```text
--a-b--c-|
```
This means:
- Wait 2 frames and `a` is issued
- `b` after 1 frame
- `c` 2 frames later
- Complete after 1 more frame

## Difference between Cold and Hot

### Cold Observable

Cold Observable is "played from the beginning with each subscription".

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

      expectObservable(source$).toBe('--a--b--c|');
    });
  });
});

```

### Hot Observable

A Hot Observable is a stream that is "already in progress".
If you subscribe in the middle of a stream, you will only receive values from that point onward.

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
    testScheduler.run(({ hot, expectObservable }) => {
      const source$ = hot('--a--b--c|');

      expectObservable(source$, '----^').toBe('-----b--c|');
    });
  });
});

```

## A simple example of a marble test

For example, to test the `debounceTime` operator

```ts
import { debounceTime } from 'rxjs';
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
      const source$ = cold('--a--b----c|');
      const result$ = source$.pipe(
        debounceTime(20)
      );

      const expected =    '-----------(c|)';

      expectObservable(result$).toBe(expected, { c: 'c' });
    });
  });
});

```

Here we verify that only the last `c` issued is output.

## Notes

- One character in marble notation represents **one frame (10ms)** by default (configurable depending on environment)
- Time-dependent operators such as `debounceTime`, `delay`, `interval`, etc. **work well with marble tests**
- Use `expectObservable` to validate the output of the stream
- `expectSubscriptions` is an advanced feature that validates the timing of subscriptions, but is not covered here

## Summary

Marble testing is a very powerful technique that makes testing RxJS code **visible and intuitive**.

- **Aware of the difference between Cold and Hot**
- **Representing time passing and events as strings**
- **Complex asynchronous streams can also be tested clearly**

Let's start with a simple marble test to practice!
