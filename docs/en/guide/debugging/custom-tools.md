---
description: This page explains how to create custom tools for RxJS debugging. It provides examples of practical debugging tool implementations, such as named stream tracking, configurable debug operators, and performance measurement operators.
---

# Custom Debugging Tools

Creating your own debugging tools allows for flexible debugging tailored to your project requirements.

## Debugging Named Streams

Create a custom operator that can name and track an Observable.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Map to manage stream names
const namedStreams = new Map<string, any[]>();

/**
 * Name and track an Observable
 */
function tagStream<T>(name: string) {
  if (!namedStreams.has(name)) {
    namedStreams.set(name, []);
  }

  return tap<T>({
    next: value => {
      const log = {
        name,
        type: 'next',
        value,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] next:`, value);
    },
    error: error => {
      const log = {
        name,
        type: 'error',
        error,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.error(`[${name}] error:`, error);
    },
    complete: () => {
      const log = {
        name,
        type: 'complete',
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] complete`);
    }
  });
}

/**
 * Get logs for a specific named stream
 */
function getStreamLogs(name: string) {
  return namedStreams.get(name) || [];
}

/**
 * Get a list of all named streams
 */
function getAllStreamNames() {
  return Array.from(namedStreams.keys());
}

/**
 * Clear logs
 */
function clearStreamLogs(name?: string) {
  if (name) {
    namedStreams.set(name, []);
  } else {
    namedStreams.clear();
  }
}
```

### Examples of use

Here is an example of naming and tracking an Observable.

```ts
import { interval } from 'rxjs';
import { map, take } from 'rxjs';

// Name the Observable
interval(1000)
  .pipe(
    tagStream('interval-stream'),
    map(x => x * 2),
    take(5)
  )
  .subscribe();

// Check logs after 3 seconds
setTimeout(() => {
  console.log('All streams:', getAllStreamNames());
  console.log('interval-stream logs:', getStreamLogs('interval-stream'));
}, 3000);

// Output:
// [interval-stream] next: 0
// [interval-stream] next: 1
// [interval-stream] next: 2
// All streams: ['interval-stream']
// interval-stream logs: [
//   { name: 'interval-stream', type: 'next', value: 0, timestamp: 1697280000000 },
//   { name: 'interval-stream', type: 'next', value: 1, timestamp: 1697280001000 },
//   { name: 'interval-stream', type: 'next', value: 2, timestamp: 1697280002000 }
// ]
```

### Track multiple streams

Name and manage multiple streams.

```ts
import { interval, fromEvent } from 'rxjs';
import { map, take } from 'rxjs';

// Name multiple streams
interval(1000)
  .pipe(
    tagStream('timer-stream'),
    map(x => x * 2),
    take(3)
  )
  .subscribe();

const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tagStream('click-stream'),
      take(5)
    )
    .subscribe();
}

// Check all streams
console.log('Tracking streams:', getAllStreamNames());
// Output: ['timer-stream', 'click-stream']
```

> [!NOTE]
> **About rxjs-spy**
>
> `rxjs-spy` was a useful library for debugging Observables, but it is no longer maintained and has compatibility issues with the latest RxJS.
>
> Instead, we recommend using custom debug operators as shown above. They are more flexible and can be customized to your project requirements.

## Create custom debug operator

Creating your own debug operators allows for greater debugging flexibility.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

interface DebugOptions {
  enabled?: boolean;
  label?: string;
  logValues?: boolean;
  logErrors?: boolean;
  logComplete?: boolean;
  logTimestamp?: boolean;
}

/**
 * Custom debug operator
 */
function debug<T>(options: DebugOptions = {}) {
  const {
    enabled = true,
    label = 'Debug',
    logValues = true,
    logErrors = true,
    logComplete = true,
    logTimestamp = false
  } = options;

  if (!enabled) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => {
      if (logValues) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] next:`, value);
      }
    },
    error: error => {
      if (logErrors) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.error(`${timestamp} [${label}] error:`, error);
      }
    },
    complete: () => {
      if (logComplete) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] complete`);
      }
    }
  });
}

// Usage example
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    debug({ label: 'Input', logTimestamp: true }),
    map(x => x * 2),
    debug({ label: 'After Map', logTimestamp: true })
  )
  .subscribe();

// Output:
// [2025-10-14T12:00:00.000Z] [Input] next: 1
// [2025-10-14T12:00:00.001Z] [After Map] next: 2
// [2025-10-14T12:00:00.001Z] [Input] next: 2
// [2025-10-14T12:00:00.002Z] [After Map] next: 4
// [2025-10-14T12:00:00.002Z] [Input] next: 3
// [2025-10-14T12:00:00.003Z] [After Map] next: 6
// [2025-10-14T12:00:00.003Z] [Input] complete
// [2025-10-14T12:00:00.004Z] [After Map] complete
```

## Debug operator for performance measurement

A performance measurement operator that automatically records execution time and number of values.

```ts
import { tap } from 'rxjs';

function measure<T>(label: string) {
  let startTime: number;
  let count = 0;

  return tap<T>({
    subscribe: () => {
      startTime = performance.now();
      console.log(`⏱️ [${label}] Start`);
    },
    next: value => {
      count++;
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Value #${count} (${elapsed.toFixed(2)}ms):`, value);
    },
    complete: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Complete (Total: ${elapsed.toFixed(2)}ms, ${count} values)`);
    },
    error: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Error (${elapsed.toFixed(2)}ms)`);
    }
  });
}

// Usage example
import { interval } from 'rxjs';
import { take, delay } from 'rxjs';

interval(100)
  .pipe(
    take(5),
    measure('Interval Stream'),
    delay(50)
  )
  .subscribe();

// Output:
// ⏱️ [Interval Stream] Start
// ⏱️ [Interval Stream] Value #1 (150.23ms): 0
// ⏱️ [Interval Stream] Value #2 (250.45ms): 1
// ⏱️ [Interval Stream] Value #3 (350.67ms): 2
// ⏱️ [Interval Stream] Value #4 (450.89ms): 3
// ⏱️ [Interval Stream] Value #5 (551.12ms): 4
// ⏱️ [Interval Stream] Complete (Total: 551.12ms, 5 values)
```

## Summary

By creating custom debugging tools

- ✅ **Named Streams** - identify and track multiple streams by name
- ✅ **Flexible configuration** - Debug operator tailored to project requirements
- ✅ **Performance Measurement** - Automatically record run time and number of values
- ✅ **Log Management** - log and retrieve time-stamped logs

It is recommended that these tools be enabled only in the development environment and disabled in the production environment.

## Related Pages

- [Basic Debugging Strategies](/en/guide/debugging/) - How to use tap operator and developer tools
- [Common Debugging Scenarios](/en/guide/debugging/common-scenarios) - Problem-specific troubleshooting
- [Performance Debugging](/en/guide/debugging/performance) - Subscription monitoring, memory usage checking
