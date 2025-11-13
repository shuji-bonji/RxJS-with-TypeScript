---
description: This page provides practical explanations of RxJS debugging techniques from the perspectives of basic strategies, common debugging scenarios, debugging tools, and performance debugging.
---

# RxJS Debugging Techniques

Due to the nature of asynchronous streams, debugging RxJS requires a different approach than traditional synchronous debugging techniques.

This page provides basic strategies for debugging RxJS applications and navigation to detailed debugging techniques.

## Debugging Techniques Overview

Debugging RxJS can be categorized into the following four approaches

| Approach | Content | Detail Page |
|----------|------|-----------|
| **Basic Strategy** | tap operator, developer tools, RxJS DevTools | Explained on this page |
| **Common Scenarios** | Six typical problems: no values flow, memory leaks, missing errors, etc. | [→ Details](/en/guide/debugging/common-scenarios) |
| **Custom Tools** | Named streams, debug operators, performance measurement | [→ Details](/en/guide/debugging/custom-tools) |
| **Performance** | Subscription monitoring, reevaluation detection, memory usage checking, best practices | [→ Details](/en/guide/debugging/performance) |

## Basic debugging strategies

### 1. log output with `tap` operator

The `tap` operator is the most basic debugging technique, allowing you to observe stream values without side-effects.

```ts
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

interval(1000)
  .pipe(
    tap(value => console.log('🔵 Original value:', value)),
    map(x => x * 2),
    tap(value => console.log('🟢 After map:', value)),
    filter(x => x > 5),
    tap(value => console.log('🟡 After filter:', value))
  )
  .subscribe(value => console.log('✅ Final value:', value));

// Output:
// 🔵 Original value: 0
// 🟢 After map: 0
// 🔵 Original value: 1
// 🟢 After map: 2
// 🔵 Original value: 2
// 🟢 After map: 4
// 🔵 Original value: 3
// 🟢 After map: 6
// 🟡 After filter: 6
// ✅ Final value: 6
```

#### Key Points
- Insert a `tap` at each step of the pipeline to track the flow of data
- Use pictograms and labels to improve the visibility of the log
- Debug logs can be safely inserted because `tap` does not change values

### 2. Output detailed log information

To obtain more detailed debugging information, use the Observer object.

```ts
import { of, throwError, concat } from 'rxjs';
import { tap } from 'rxjs';

const debug = (tag: string) =>
  tap({
    next: value => console.log(`[${tag}] next:`, value),
    error: error => console.error(`[${tag}] error:`, error),
    complete: () => console.log(`[${tag}] complete`)
  });

// Normal stream
of(1, 2, 3)
  .pipe(debug('Normal'))
  .subscribe();

// Output:
// [Normal] next: 1
// [Normal] next: 2
// [Normal] next: 3
// [Normal] complete

// Stream with error
concat(
  of(1, 2),
  throwError(() => new Error('Error occurred'))
)
  .pipe(debug('Error'))
  .subscribe({
    error: () => {} // Error handling
  });

// Output:
// [Error] next: 1
// [Error] next: 2
// [Error] error: Error: Error occurred
```

### 3. check with developer tools

This is a debugging technique that utilizes the browser's developer tools.

```ts
import { fromEvent, timer } from 'rxjs';
import { map, tap, debounceTime } from 'rxjs';

// Helper function for debugging
function tapDebugger<T>(label: string) {
  return tap<T>({
    next: value => {
      console.group(`🔍 ${label}`);
      console.log('Value:', value);
      console.log('Type:', typeof value);
      console.log('Timestamp:', new Date().toISOString());
      console.trace('Stack trace');
      console.groupEnd();
    }
  });
}

// Debugging button click events
const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tapDebugger('Click Event'),
      debounceTime(300),
      tapDebugger('After Debounce'),
      map(() => ({ timestamp: Date.now() }))
    )
    .subscribe(data => console.log('📤 Send:', data));
}
```

#### Utilizing Developer Tools
- Group logs with `console.group()`
- Display stack traces with `console.trace()`
- Display arrays and objects in an easy-to-read format with `console.table()`
- Place breakpoints in `tap`

### 4. Utilizing RxJS DevTools

RxJS DevTools is a debugging tool provided as a browser extension.

#### Installation
- Chrome: [RxJS DevTools - Chrome Web Store](https://chrome.google.com/webstore)
- Firefox: [RxJS DevTools - Firefox Add-ons](https://addons.mozilla.org/)

#### Main features
- Visualization of Observable subscription status
- Timeline display of stream values
- Memory leak detection
- Performance analysis

#### Usage Example

```ts
import { interval } from 'rxjs';
import { take, map } from 'rxjs';

// Enable debugging only in development environment
// Different build tools use different environment variable checks
const isDevelopment =
  // Vite: import.meta.env.DEV
  // webpack: process.env.NODE_ENV === 'development'
  // Manual setup: use global variables
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

const stream$ = interval(1000).pipe(
  take(5),
  map(x => x * 2)
);

if (isDevelopment) {
  // Make observable in DevTools
  stream$.subscribe({
    next: value => console.log('DevTools:', value)
  });
}
```

## Detailed debugging techniques

Once you understand the basic strategy, learn specific debugging techniques on the detailed pages below.

### Common Debugging Scenarios

Six typical problems encountered in real-world development and how to solve them

- Scenario 1: No values flow
- Scenario 2: Different value than expected is output
- Scenario 3: Subscription never completes (infinite stream)
- Scenario 4: Memory leak (forgot to unsubscribe)
- Scenario 5: Error occurs and is not noticed
- Scenario 6: I want to track retry attempts

[→ View Common Debugging Scenarios](/en/guide/debugging/common-scenarios)

### Custom Debug Tools

How to create your own debugging tools to meet your project requirements

- Debugging Named Streams (tagStream)
- Creating custom debug operators
- Operator for performance measurement (measure)

[→ View Custom Debug Tools](/en/guide/debugging/custom-tools)

### Performance Debugging

Application Optimization and Best Practices

- Check and track subscriptions
- Detect unnecessary reevaluations (shareReplay)
- Monitor memory usage
- Creating a Debugging Environment
- Type-safe debugging
- Set error bounds

[→ View Performance Debugging](/en/guide/debugging/performance)

## Summary

Debugging RxJS can be done efficiently by following these points.

### Basic Strategy
- ✅ Observe each stage of the stream with the `tap` operator
- ✅ Utilize developer tools for detailed log output
- ✅ Visualize the stream with RxJS DevTools

### Common Scenarios
- ✅ Values do not flow → Forgot subscription, check filtering conditions
- ✅ Different value than expected → Operator order, note reference sharing
- ✅ Subscription not completed → use `take` or `takeUntil` for infinite streams
- ✅ Memory leaks → auto unsubscribe with `takeUntil` pattern
- ✅ Missing errors → implement proper error handling

### Debugging Tools
- ✅ Flexible debugging with custom debug operators
- ✅ Track multiple streams with named streams
- ✅ Identify bottlenecks with performance measurement

### Performance
- ✅ Prevent memory leaks by monitoring subscriptions
- ✅ Avoid unnecessary recalculations with `shareReplay`
- ✅ Check memory usage periodically

Combined, these techniques allow for efficient debugging of RxJS applications.

## Related Pages

- [Error Handling](/en/guide/error-handling/strategies) - Error handling strategies
- [Testing Techniques](/en/guide/testing/unit-tests) - How to test RxJS
- [RxJS Anti-patterns](/en/guide/anti-patterns/) - Common mistakes and solutions
- [Pipeline](/en/guide/operators/pipeline) - Chaining operators
