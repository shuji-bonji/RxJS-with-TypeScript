---
description: Utility operators are a group of auxiliary operators in RxJS that are responsible for controlling side effects, delay processing, subscription management, etc.
---

# Utility Operators

Utility operators in RxJS are a group of operators that are responsible for **auxiliary processing of streams (side effects, state control, UI support, etc.)** rather than the main purpose of data conversion or filtering.

On this page, operators are categorized by purpose as shown below, and a list is provided to confirm their basic usage.
For detailed usage and practical examples, please refer to the respective pages or [Practical Use Cases](./practical-use-cases.md).


## List of Operators (by Purpose)

### ◾ Side Effects and State Control

| Operator | Description | Often Combined With |
|--------------|------|------------------|
| [tap](./tap.md) | Execute side effects without changing values (log output, UI updates, etc.) | `map`, `switchMap` |
| [finalize](./finalize.md) | Execute cleanup processing when the stream ends | `tap`, `catchError` |


### ◾ Timing and Delay Control

| Operator | Description | Often Combined With |
|--------------|------|------------------|
| [delay](./delay.md) | Delay the emission of each value by a specified time | `tap`, `concatMap` |
| [timeout](./timeout.md) | Generate an error if emission exceeds a certain time | `catchError`, `retry` |
| [takeUntil](./takeUntil.md) | End subscription when the specified Observable notifies | `interval`, `fromEvent` |


### ◾ Initial Value, Repetition, Array Conversion, etc.

| Operator | Description | Often Combined With |
|--------------|------|------------------|
| [startWith](./startWith.md) | Emit an initial value at the beginning of the stream | `scan`, `combineLatest` |
| [repeat](./repeat.md) | Resubscribe to the entire stream after completion | `tap`, `delay` |
| [retry](./retry.md) | Retry on error | `catchError`, `switchMap` |
| [toArray](./toArray.md) | Emit all values in the stream as a single array (on completion) | `concatMap`, `take` |


## Remarks

- Difference between `retry` and `repeat`:
  - `retry`: **Retry on error**
  - `repeat`: **Retry on successful completion**
- `toArray` does not output a value unless it completes, so it is commonly used with `take()` and so on.
