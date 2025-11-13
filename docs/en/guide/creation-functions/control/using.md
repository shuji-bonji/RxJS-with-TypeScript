---
description: using() is an RxJS Creation Function that automatically creates and releases resources according to the Observable lifecycle. It safely manages resources that require manual cleanup such as WebSockets, file handles, and timers, and prevents memory leaks. Ensures reliable resource management by creating resources at the start of a subscription and automatically releasing them at the end of a subscription.
---

# using()

[📘 RxJS Official Documentation - using](https://rxjs.dev/api/index/function/using)

`using()` is a Creation Function that automatically creates and deallocates resources according to the Observable lifecycle, safely managing resources that need to be cleaned up manually, such as WebSockets, file handles, and timers, and prevents memory leaks.

## Basic usage

### Simple resource management

```typescript
import { using, interval, Subscription, take } from 'rxjs';
const resource$ = using(
  // Resource factory: executed at the start of subscription
  () => {
    console.log('Resource created');
    return new Subscription(() => console.log('Resource released'));
  },
  // Observable factory: create Observable using resource
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Value:', value),
  complete: () => console.log('Complete')
});

// Output:
// Resource created
// Value: 0
// Value: 1
// Value: 2
// Complete
// Resource released
```

> [!IMPORTANT]
> **Automatic resource release**
>
> `using()` automatically releases resources when the Observable completes (`complete`) or is unsubscribed (`unsubscribe`).

## how using() works

`using()` takes the following two functions.

```typescript
function using<T>(
  resourceFactory: () => Unsubscribable | void,
  observableFactory: (resource: Unsubscribable | void) => ObservableInput<T>
): Observable<T>
```

### 1. resourceFactory

Runs at the start of a subscription to create a resource. Must return an object with a `unsubscribe()` method.

```typescript
// Return a Subscription
() => new Subscription(() => {
  console.log('Cleanup processing');
});

// Or return an object with an unsubscribe method
() => ({
  unsubscribe: () => {
    console.log('Cleanup processing');
  }
});
```

### 2. observableFactory

Creates an Observable with a resource.

```typescript
(resource) => interval(1000);
```

## Practical Patterns

### Managing WebSocket connections

```typescript
import { using, interval, Subject, map, takeUntil } from 'rxjs';
function createWebSocketStream(url: string) {
  return using(
    // Create WebSocket connection
    () => {
      const ws = new WebSocket(url);
      console.log('WebSocket connection started:', url);

      ws.onopen = () => console.log('Connection complete');
      ws.onerror = (error) => console.error('Connection error:', error);

      return {
        unsubscribe: () => {
          console.log('WebSocket connection closed');
          ws.close();
        }
      };
    },
    // Create message stream
    () => {
      const messages$ = new Subject<MessageEvent>();
      const ws = new WebSocket(url);

      ws.onmessage = (event) => messages$.next(event);
      ws.onerror = (error) => messages$.error(error);
      ws.onclose = () => messages$.complete();

      return messages$;
    }
  );
}

// Usage example
const websocket$ = createWebSocketStream('wss://echo.websocket.org');

const subscription = websocket$.subscribe({
  next: message => console.log('Received:', message.data),
  error: error => console.error('Error:', error),
  complete: () => console.log('Complete')
});

// Automatically close WebSocket after 10 seconds
setTimeout(() => subscription.unsubscribe(), 10000);
```

### Automatic timer cleanup

```typescript
import { using, Observable, Subscription } from 'rxjs';

function createTimerStream(intervalMs: number) {
  return using(
    // Create timer resource
    () => {
      let timerId: number | null = null;
      console.log('Timer started');

      return new Subscription(() => {
        if (timerId !== null) {
          clearInterval(timerId);
          console.log('Timer stopped');
        }
      });
    },
    // Create timer stream
    () => new Observable(subscriber => {
      const timerId = setInterval(() => {
        subscriber.next(Date.now());
      }, intervalMs);

      return () => clearInterval(timerId);
    })
  );
}

// Usage example
const timer$ = createTimerStream(1000);

const subscription = timer$.subscribe({
  next: time => console.log('Current time:', new Date(time).toLocaleTimeString())
});

// Stop after 5 seconds
setTimeout(() => subscription.unsubscribe(), 5000);
```

### File manipulation (Node.js)

```typescript
import { using, Observable } from 'rxjs';
import * as fs from 'fs';

function readFileStream(filePath: string) {
  return using(
    // Open file handle
    () => {
      const fd = fs.openSync(filePath, 'r');
      console.log('File opened:', filePath);

      return {
        unsubscribe: () => {
          fs.closeSync(fd);
          console.log('File closed');
        }
      };
    },
    // Create file read stream
    () => new Observable<string>(subscriber => {
      const stream = fs.createReadStream(filePath, { encoding: 'utf8' });

      stream.on('data', (chunk) => subscriber.next(chunk));
      stream.on('error', (error) => subscriber.error(error));
      stream.on('end', () => subscriber.complete());

      return () => stream.destroy();
    })
  );
}

// Usage example
const file$ = readFileStream('./data.txt');

file$.subscribe({
  next: chunk => console.log('Reading:', chunk),
  error: error => console.error('Error:', error),
  complete: () => console.log('Read complete')
});
```

### Managing event listeners

```typescript
import { using, Observable } from 'rxjs';

function createClickStream(element: HTMLElement) {
  return using(
    // Register event listener
    () => {
      console.log('Event listener registered');

      return {
        unsubscribe: () => {
          console.log('Event listener removed');
          // Actual removal is done in the Observable factory
        }
      };
    },
    // Create click event stream
    () => new Observable<MouseEvent>(subscriber => {
      const handler = (event: MouseEvent) => subscriber.next(event);

      element.addEventListener('click', handler);

      return () => {
        element.removeEventListener('click', handler);
      };
    })
  );
}

// Usage example
const button = document.querySelector('#myButton') as HTMLElement;
const clicks$ = createClickStream(button);

const subscription = clicks$.subscribe({
  next: event => console.log('Click position:', event.clientX, event.clientY)
});

// Auto-remove after 30 seconds
setTimeout(() => subscription.unsubscribe(), 30000);
```

## Common use cases

### 1. database connection management

```typescript
import { using, from, mergeMap } from 'rxjs';
interface DbConnection {
  query: (sql: string) => Promise<any[]>;
  close: () => Promise<void>;
}

function queryWithConnection(sql: string) {
  return using(
    // Establish database connection
    () => {
      const connection = createDbConnection();
      console.log('DB connection established');

      return {
        unsubscribe: async () => {
          await connection.close();
          console.log('DB connection closed');
        }
      };
    },
    // Execute query
    () => {
      const connection = createDbConnection();
      return from(connection.query(sql));
    }
  );
}

// Usage example
const users$ = queryWithConnection('SELECT * FROM users');

users$.subscribe({
  next: rows => console.log('Fetched:', rows),
  error: error => console.error('Error:', error),
  complete: () => console.log('Query complete')
});

function createDbConnection(): DbConnection {
  // Actual connection processing
  return {
    query: async (sql) => [],
    close: async () => {}
  };
}
```

### 2. Resource pool management

```typescript
import { using, Observable, defer } from 'rxjs';

class ResourcePool<T> {
  private available: T[] = [];
  private inUse = new Set<T>();

  constructor(private factory: () => T, size: number) {
    for (let i = 0; i < size; i++) {
      this.available.push(factory());
    }
  }

  acquire(): T | null {
    const resource = this.available.pop();
    if (resource) {
      this.inUse.add(resource);
      return resource;
    }
    return null;
  }

  release(resource: T): void {
    if (this.inUse.has(resource)) {
      this.inUse.delete(resource);
      this.available.push(resource);
    }
  }
}

// Usage example
const pool = new ResourcePool(() => ({ id: Math.random() }), 5);

function usePooledResource<T>(
  pool: ResourcePool<T>,
  work: (resource: T) => Observable<any>
) {
  return using(
    () => {
      const resource = pool.acquire();
      if (!resource) {
        throw new Error('Resource pool exhausted');
      }
      console.log('Resource acquired:', resource);

      return {
        unsubscribe: () => {
          pool.release(resource);
          console.log('Resource returned:', resource);
        }
      };
    },
    (subscription) => {
      const resource = pool.acquire();
      return resource ? work(resource) : defer(() => {
        throw new Error('Resource acquisition failed');
      });
    }
  );
}

// Process using resource
const work$ = usePooledResource(pool, (resource) =>
  new Observable(subscriber => {
    subscriber.next(`Processing: ${resource.id}`);
    setTimeout(() => subscriber.complete(), 1000);
  })
);

work$.subscribe({
  next: result => console.log(result),
  complete: () => console.log('Processing complete')
});
```

### 3. multiple resource coordination management

```typescript
import { using, merge, Subject } from 'rxjs';

interface MultiResource {
  ws: WebSocket;
  timer: number;
}

function createMultiResourceStream() {
  return using(
    // Create multiple resources
    () => {
      const ws = new WebSocket('wss://echo.websocket.org');
      const timer = setInterval(() => {
        console.log('Periodic execution');
      }, 1000);

      console.log('Multiple resources created');

      return {
        unsubscribe: () => {
          ws.close();
          clearInterval(timer);
          console.log('Multiple resources released');
        }
      };
    },
    // Combine multiple streams
    () => {
      const messages$ = new Subject<string>();
      const ticks$ = new Subject<number>();

      return merge(messages$, ticks$);
    }
  );
}

// Usage example
const multiStream$ = createMultiResourceStream();

const subscription = multiStream$.subscribe({
  next: value => console.log('Received:', value)
});

// Release all resources after 10 seconds
setTimeout(() => subscription.unsubscribe(), 10000);
```

### 4. Conditional resource management

```typescript
import { using, interval, EMPTY, take } from 'rxjs';
function conditionalResource(shouldCreate: boolean) {
  return using(
    () => {
      if (shouldCreate) {
        console.log('Resource created');
        return {
          unsubscribe: () => console.log('Resource released')
        };
      } else {
        console.log('Resource creation skipped');
        return { unsubscribe: () => {} };
      }
    },
    () => {
      if (shouldCreate) {
        return interval(1000).pipe(take(3));
      } else {
        return EMPTY;
      }
    }
  );
}

// When creating resources
conditionalResource(true).subscribe({
  next: val => console.log('Value:', val),
  complete: () => console.log('Complete')
});

// When not creating resources
conditionalResource(false).subscribe({
  next: val => console.log('Value:', val),
  complete: () => console.log('Complete')
});
```

## Error handling

### Resource release on error

```typescript
import { using, throwError, of, catchError } from 'rxjs';
const errorHandling$ = using(
  () => {
    console.log('Resource created');
    return {
      unsubscribe: () => console.log('Resource released (executed even on error)')
    };
  },
  () => throwError(() => new Error('Intentional error'))
);

errorHandling$.pipe(
  catchError(error => {
    console.error('Error caught:', error.message);
    return of('Default value');
  })
).subscribe({
  next: val => console.log('Value:', val),
  complete: () => console.log('Complete')
});

// Output:
// Resource created
// Resource released (executed even on error)
// Error caught: Intentional error
// Value: Default value
// Complete
```

> [!IMPORTANT]
> **Reliable resource release even on error**
>
> `using()` always releases the resource created in `resourceFactory`, even if an error occurs.

## Common errors and how to handle them

### 1. forgot to implement unsubscribe method

**Error example:**
```typescript
// ❌ Error: no unsubscribe method
using(
  () => {
    console.log('Resource created');
    return {}; // no unsubscribe
  },
  () => interval(1000)
);
```

**Solution:**
```typescript
// ✅ Correct: implement unsubscribe method
using(
  () => {
    console.log('Resource created');
    return {
      unsubscribe: () => console.log('Resource released')
    };
  },
  () => interval(1000)
);
```

### 2. Creating asynchronous resources

**Problem:**
```typescript
// ❌ Problem: resourceFactory cannot be asynchronous
using(
  async () => { // async cannot be used
    const resource = await createResourceAsync();
    return resource;
  },
  () => interval(1000)
);
```

**Solution:**
```typescript
import { defer, from, mergeMap } from 'rxjs';
// ✅ Correct: handle asynchronous processing with defer and mergeMap

defer(() =>
  from(createResourceAsync()).pipe(
    mergeMap(resource =>
      using(
        () => resource,
        () => interval(1000)
      )
    )
  )
);
```

### 3. Duplicate resource creation

**Problem:**
```typescript
// ❌ Problem: create resources separately in resourceFactory and observableFactory
let sharedResource: any;

using(
  () => {
    sharedResource = createResource(); // Create here
    return { unsubscribe: () => sharedResource.close() };
  },
  () => {
    const resource = createResource(); // Create again
    return from(resource.getData());
  }
);
```

**Solution:**
```typescript
// ✅ Correct: share resources
using(
  () => {
    const resource = createResource();
    return {
      resource, // Hold resource
      unsubscribe: () => resource.close()
    };
  },
  (subscription: any) => {
    return from(subscription.resource.getData());
  }
);
```

## best practices for using()

### 1. Ensure resource release

```typescript
// ✅ Good example: try-finally pattern
using(
  () => {
    const resource = createResource();
    return {
      unsubscribe: () => {
        try {
          resource.close();
        } catch (error) {
          console.error('Resource release error:', error);
        }
      }
    };
  },
  () => interval(1000)
);
```

### 2. Resource creation logging

```typescript
// ✅ Good example: log resource lifecycle
using(
  () => {
    const resourceId = Math.random();
    console.log(`[${resourceId}] Resource created`);

    return {
      unsubscribe: () => {
        console.log(`[${resourceId}] Resource released`);
      }
    };
  },
  () => interval(1000)
);
```

### 3. Type-safe resource management

```typescript
// ✅ Good example: utilize TypeScript types
interface ManagedResource {
  id: string;
  close: () => void;
}

function createManagedStream(resource: ManagedResource) {
  return using(
    () => {
      console.log('Resource started:', resource.id);
      return {
        unsubscribe: () => {
          resource.close();
          console.log('Resource ended:', resource.id);
        }
      };
    },
    () => interval(1000)
  );
}
```

## Comparison with manual management

### Manual resource management (❌ not recommended)

```typescript
// ❌ Bad example: manual management (risk of forgetting to release)
const ws = new WebSocket('wss://example.com');
const subscription = interval(1000).subscribe(() => {
  ws.send('ping');
});

// May forget to release
// subscription.unsubscribe();
// ws.close();
```

### Resource management by using() (✅ recommended)

```typescript
// ✅ Good example: automatic management with using()
const stream$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return {
      unsubscribe: () => ws.close()
    };
  },
  () => interval(1000)
);

const subscription = stream$.subscribe(() => {
  // Processing using WebSocket
});

// WebSocket is also automatically closed with unsubscribe()
subscription.unsubscribe();
```

## Summary

`using()` is a Creation Function that automatically manages resources according to the Observable's life cycle.

**Key Features:**
- Creates a resource at the start of a subscription
- Automatic release at end of subscription (complete or unsubscribe)
- Prevents memory leaks
- Reliable resource release even on error

**use cases:**
- Network connections such as WebSocket, EventSource
- File handles, database connections
- Automatic cleanup of timers and intervals
- Automatic deactivation of event listeners

**Note:**
- `resourceFactory` must be a synchronous function
- Always implement the `unsubscribe` method
- Ensure proper error handling.

**Recommended usage:**
- Avoid forgetting to release resources
- Log the lifecycle
- Utilize TypeScript types for type-safe management

## Related Pages

- [scheduled()](/en/guide/creation-functions/control/scheduled) - Generate Observable with specified scheduler
- [Control Creation Functions](/en/guide/creation-functions/control/) - Comparison of scheduled() and using()
- [finalize()](/en/guide/error-handling/finalize) - Operator to add processing at end of subscription

## References

- [RxJS Official Documentation - using](https://rxjs.dev/api/index/function/using)
- [RxJS Official Documentation - Subscription](https://rxjs.dev/guide/subscription)
