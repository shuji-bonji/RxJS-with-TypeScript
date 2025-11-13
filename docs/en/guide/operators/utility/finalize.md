---
description: finalize is an RxJS utility operator that defines a process to be executed whenever an Observable is completed, errored, or unsubscribed. It is ideal for situations that require cleanup at end-of-stream, such as resource release, end of loading display, and cleanup operations. It ensures that operations are executed as reliably as try-finally and helps prevent memory leaks.
---

# finalize - Processing on Completion

The `finalize` operator defines a process that is called whenever **Observable is completed, errors, or is unsubscribed**.
This is ideal for "must execute" processes such as cleanup and UI loading release.

## 🔰 Basic Syntax and Operation

```ts
import { of } from 'rxjs';
import { finalize } from 'rxjs';

of('Completed')
  .pipe(finalize(() => console.log('Stream has ended')))
  .subscribe(console.log);
// Output:
// Completed
// Stream has ended
```

In this example, the process in `finalize` is executed after emitting one value in `of()`.
**It is called reliably for both `complete` and `error`**.

[🌐 RxJS Official Documentation - finalize](https://rxjs.dev/api/index/function/finalize)

## 💡 Typical Usage Example

The following is an example of switching loading display before and after streaming.

```ts
import { of } from 'rxjs';
import { tap, delay, finalize } from 'rxjs';

let isLoading = false;

of('Data')
  .pipe(
    tap(() => {
      isLoading = true;
      console.log('Loading started');
    }),
    delay(1000),
    finalize(() => {
      isLoading = false;
      console.log('Loading ended');
    })
  )
  .subscribe((value) => console.log('Retrieved:', value));
// Output:
// Loading started
// Retrieved: Data
// Loading ended
```

## 🧪 Practical Code Example (with UI)

```ts
import { interval } from 'rxjs';
import { take, finalize, tap } from 'rxjs';

// Output display area
const finalizeOutput = document.createElement('div');
finalizeOutput.innerHTML = '<h3>finalize Example:</h3>';
document.body.appendChild(finalizeOutput);

// Loading indicator
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Loading data...';
loadingIndicator.style.backgroundColor = '#e0f7fa';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.borderRadius = '5px';
finalizeOutput.appendChild(loadingIndicator);

// Progress display
const progressContainer = document.createElement('div');
progressContainer.style.marginTop = '10px';
finalizeOutput.appendChild(progressContainer);

// Completion message element
const completionMessage = document.createElement('div');
completionMessage.style.marginTop = '10px';
completionMessage.style.fontWeight = 'bold';
finalizeOutput.appendChild(completionMessage);

// Data retrieval simulation
interval(500)
  .pipe(
    take(5), // Retrieve 5 values
    tap((val) => {
      const progressItem = document.createElement('div');
      progressItem.textContent = `Processing item ${val + 1}...`;
      progressContainer.appendChild(progressItem);
    }),
    finalize(() => {
      loadingIndicator.style.display = 'none';
      completionMessage.textContent = 'Processing completed!';
      completionMessage.style.color = 'green';
    })
  )
  .subscribe({
    complete: () => {
      const successMsg = document.createElement('div');
      successMsg.textContent = 'All data loaded successfully.';
      completionMessage.appendChild(successMsg);
    },
  });
```

## ✅ Summary

- `finalize` is **always executed** regardless of completion, error, or manual termination
- Ideal for cleanup and loading termination processes
- Can be combined with other operators (`tap`, `delay`, etc.) for **safe asynchronous cleanup**
