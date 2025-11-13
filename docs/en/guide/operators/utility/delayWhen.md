---
description: The delayWhen operator dynamically controls the delay timing of each value with a separate Observable to achieve flexible delay processing according to conditions.
---

# delayWhen - Dynamic Delay Control

The `delayWhen` operator dynamically determines the delay time for each value **with an individual Observable**. Whereas the `delay` operator provides a fixed time delay, `delayWhen` can apply a different delay for each value.

## 🔰 Basic Syntax and Operation

Specifies a function that returns an Observable that determines the delay for each value.

```ts
import { of, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    delayWhen(value => {
      const delayTime = value === 'B' ? 2000 : 1000;
      return timer(delayTime);
    })
  )
  .subscribe(console.log);
// Output:
// A (after 1 second)
// C (after 1 second)
// B (after 2 seconds)
```

In this example, only the value `'B'` will have a 2 second delay applied, the others will have a 1 second delay.

[🌐 RxJS Official Documentation - delayWhen](https://rxjs.dev/api/index/function/delayWhen)

## 💡 Typical Usage Examples

- **Value-based delay**: Change delay based on priority or type
- **Delay based on external events**: Wait for user interaction or completion of other streams
- **Conditional delay**: Delay for a specific value only
- **Asynchronous timing control**: Wait for API response or data readiness

## 🧪 Practical Code Example 1: Delay by Priority

This is an example of controlling processing timing according to task priority.

```ts
import { from, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

// UI creation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'delayWhen - Priority-based delay';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

interface Task {
  id: number;
  name: string;
  priority: 'high' | 'medium' | 'low';
}

const tasks: Task[] = [
  { id: 1, name: 'Task A', priority: 'low' },
  { id: 2, name: 'Task B', priority: 'high' },
  { id: 3, name: 'Task C', priority: 'medium' },
  { id: 4, name: 'Task D', priority: 'high' },
  { id: 5, name: 'Task E', priority: 'low' }
];

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const time = now.toLocaleTimeString('en-US', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${time}] ${message}`;
  output.appendChild(logItem);
}

addLog('Processing started', '#e3f2fd');

from(tasks)
  .pipe(
    delayWhen(task => {
      // Set delay time according to priority
      let delayTime: number;
      switch (task.priority) {
        case 'high':
          delayTime = 500;  // High priority: 0.5 seconds
          break;
        case 'medium':
          delayTime = 1500; // Medium priority: 1.5 seconds
          break;
        case 'low':
          delayTime = 3000; // Low priority: 3 seconds
          break;
      }
      return timer(delayTime);
    })
  )
  .subscribe({
    next: task => {
      const colors = {
        high: '#c8e6c9',
        medium: '#fff9c4',
        low: '#ffccbc'
      };
      addLog(
        `Processing ${task.name} (priority: ${task.priority})`,
        colors[task.priority]
      );
    },
    complete: () => {
      addLog('All tasks completed', '#e3f2fd');
    }
  });
```

- High priority tasks are processed after 0.5 seconds
- Medium priority tasks are processed after 1.5 seconds, low priority after 3 seconds
- Realizes processing order according to task importance

## 🧪 Practical Code Example 2: Delay Due to External Events

This is an example of waiting for a user click before emitting a value.

```ts
import { of, fromEvent } from 'rxjs';
import { delayWhen, take, tap } from 'rxjs';

// UI creation
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'delayWhen - Click waiting';
container2.appendChild(title2);

const button = document.createElement('button');
button.textContent = 'Click to display next value';
button.style.marginBottom = '10px';
container2.appendChild(button);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.minHeight = '100px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

let clickCount = 0;

of('Message 1', 'Message 2', 'Message 3')
  .pipe(
    tap(msg => {
      addLog2(`Waiting: ${msg} (please click button)`);
      button.textContent = `Click to display "${msg}"`;
    }),
    delayWhen(() => {
      // Delay until click event occurs
      return fromEvent(button, 'click').pipe(take(1));
    })
  )
  .subscribe({
    next: msg => {
      clickCount++;
      addLog2(`✅ Displayed: ${msg}`);
      if (clickCount < 3) {
        button.disabled = false;
      } else {
        button.textContent = 'Complete';
        button.disabled = true;
      }
    },
    complete: () => {
      addLog2('--- All messages displayed ---');
    }
  });
```

- Each value is emitted after waiting for a user click
- Delay control triggered by external events is possible
- Can be applied to interactive sequence processing

## 🆚 Comparison with delay

```ts
import { of, timer } from 'rxjs';
import { delay, delayWhen } from 'rxjs';

// delay - fixed time delay
of(1, 2, 3)
  .pipe(delay(1000))
  .subscribe(console.log);
// All values delayed by 1 second

// delayWhen - different delay per value
of(1, 2, 3)
  .pipe(
    delayWhen(value => timer(value * 1000))
  )
  .subscribe(console.log);
// 1 after 1 second, 2 after 2 seconds, 3 after 3 seconds
```

| Operator | Delay Control | Use Case |
|:---|:---|:---|
| `delay` | Fixed time | Simple uniform delay |
| `delayWhen` | Dynamic (per value) | Conditional delay, external event waiting |

## ⚠️ Important Notes

### 1. Delay Observable is Newly Generated Each Time

```ts
// ❌ Bad example: Reusing same Observable instance
const delayObs$ = timer(1000);
source$.pipe(
  delayWhen(() => delayObs$)  // Won't work from 2nd time
).subscribe();

// ✅ Good example: Generate new Observable each time
source$.pipe(
  delayWhen(() => timer(1000))
).subscribe();
```

### 2. When Delay Observable Doesn't Complete

```ts
import { of, NEVER } from 'rxjs';
import { delayWhen } from 'rxjs';

// ❌ Bad example: Returning NEVER delays forever
of(1, 2, 3)
  .pipe(
    delayWhen(() => NEVER)  // Values won't be emitted
  )
  .subscribe(console.log);
// Nothing output
```

Delay Observable must always emit a value or complete.

### 3. Error Handling

If an error occurs within the delay Observable, the entire stream will error.

```ts
import { of, throwError, timer, delayWhen } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delayWhen(value => {
      if (value === 2) {
        return throwError(() => new Error('Delay error'));
      }
      return timer(1000);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Error:', err.message)
  });
// Output: 1
// Error: Delay error
```

## 📚 Related Operators

- **[delay](./delay)** - Fixed time delay
- **[debounceTime](../filtering/debounceTime)** - Delay after input stops
- **[throttleTime](../filtering/throttleTime)** - Pass through value every fixed period
- **[timeout](./timeout)** - Timeout control

## ✅ Summary

The `delayWhen` operator dynamically controls the delay timing for each value.

- ✅ Different delays can be applied to each value
- ✅ Delay control by external events and Observable
- ✅ Adjust processing timing based on priority and type
- ⚠️ Delay Observable must be newly generated each time
- ⚠️ Delay Observable must complete or emit a value
- ⚠️ Be careful with error handling
