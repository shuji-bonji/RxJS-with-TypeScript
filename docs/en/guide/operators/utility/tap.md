---
description: The tap operator is a utility operator that allows side effects to be executed without affecting the value of the stream. Ideal for debugging with log output, controlling loading states, analysis tracking, error monitoring, and other applications where external processing is performed while observing the stream. Side effects can be managed in declarative code while maintaining TypeScript's type safety.
---

# tap - Execute Side Effects

The `tap` operator is used to "execute side effects without modifying the stream."
Ideal for logging, debugging, or other operations that do not affect values.

## 🔰 Basic Syntax and Operation

Utilized in situations where you want to add only side effects without changing the value stream.

```ts
import { of, tap } from 'rxjs';

of(42).pipe(
  tap(value => console.log('tap:', value))
).subscribe();
// Output:
// tap: 42
```

In this example, the value emitted from `of(42)` is logged as it passes through `tap`.
Because `tap` passes the value "as is", it has no effect on the contents of the stream.

[🌐 RxJS Official Documentation - tap](https://rxjs.dev/api/index/function/tap)

## 💡 Typical Use Cases

`tap` is often used for the following purposes:

- Debugging and logging
- Toggling loading state
- Displaying toast notifications
- Triggering UI updates

```ts
import { of, tap, map } from 'rxjs';

of(Math.random()).pipe(
  tap(val => console.log('Retrieved value:', val)),
  map(n => n > 0.5 ? 'High' : 'Low'),
  tap(label => console.log('Label:', label))
).subscribe();
// Output:
// Retrieved value: 0.09909888881113504
// Label: Low
```


## 🧪 Practical Code Example (with UI)

The following is an example of adding logs to the DOM using tap.

```ts
import { of } from 'rxjs';
import { tap, map } from 'rxjs';

// Element for log output
const logOutput = document.createElement('div');
document.body.appendChild(logOutput);

// Value sequence
of(1, 2, 3, 4, 5)
  .pipe(
    tap((val) => {
      console.log(`Original value: ${val}`);

      // Add log to UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Value ${val} passed through`;
      logEntry.style.color = '#666';
      logOutput.appendChild(logEntry);
    }),
    map((val) => val * 10),
    tap((val) => {
      console.log(`Transformed value: ${val}`);

      // Add log to UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Transformed value ${val}`;
      logEntry.style.color = '#090';
      logOutput.appendChild(logEntry);
    })
  )
  .subscribe((val) => {
    // Display final result in UI
    const resultItem = document.createElement('div');
    resultItem.textContent = `Result: ${val}`;
    resultItem.style.fontWeight = 'bold';
    logOutput.appendChild(resultItem);
  });

```


## ✅ Summary

- `tap` is an operator specialized for **inserting side effects**
- **Log output and UI updates** can be done without changing the value flow
- Can be combined with `finalize` and `catchError` for more practical control
