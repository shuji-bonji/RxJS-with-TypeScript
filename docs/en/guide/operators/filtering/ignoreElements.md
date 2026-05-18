---
description: "The ignoreElements operator is an RxJS filtering operator that ignores all values and only passes through completions and errors. This is useful when waiting for a process to complete."
---

# ignoreElements - only completions/errors pass

The `ignoreElements` operator **ignores all values** issued from the source Observable and passes downstream **only completion and error notifications**.

## 🔰 Basic Syntax and Usage

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Value:', value), // Not called
  complete: () => console.log('Completed')
});
// Output: Completed
```

**Flow of operation**:.
1. 1, 2, 3, 4, 5 are all ignored 2.
Only completion notifications are transmitted downstream

[🌐 RxJS Official Documentation - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Typical utilization pattern

- **Waiting for the process to complete**: When you do not need the value and only want to know the completion
- **Execute only side effects**: execute side effects with tap and ignore values
- **Error Handling**: Catch only errors.
- **Synchronize sequence**: Wait for multiple processes to complete

## 🧠 Practical code example 1: Wait for completion of initialization process

This is an example of waiting for multiple initialization processes to complete.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// UICreate
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Application initialization';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Function to add status log
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Initialization process1: Database connection
const initDatabase$ = from(['DBConnecting...', 'Checking table...', 'DBReady']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Ignore values, notify only completion
);

// Initialization process2: Loading configuration file
const loadConfig$ = from(['Configuration file is being read...', 'Analyzing configuration...', 'Configuration application complete']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Initialization process3: User authentication
const authenticate$ = from(['Verifying authentication information...', 'Token verification in progress...', 'Authentication complete']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// All initialization processes are executed
addLog('Initialization started...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ All initialization is complete.！Application can be started.';
    addLog('Application startup', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Initialization error: ${err.message}`;
  }
});
```

- A detailed log of each initialization process is displayed, but the values are ignored.
- When all processes are complete, a completion message is displayed.

## 🎯 Practical Code Example 2: Waiting for file upload to complete

This is an example of displaying the upload progress of multiple files, but only notifying completion.

```ts
import { from, of, concat } from 'rxjs';
import { ignoreElements, tap, delay, mergeMap } from 'rxjs';

// UICreate
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'File upload';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Upload started';
container.appendChild(button);

const progressArea = document.createElement('div');
progressArea.style.marginTop = '10px';
container.appendChild(progressArea);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.display = 'none';
container.appendChild(result);

interface FileUpload {
  name: string;
  size: number;
}

const files: FileUpload[] = [
  { name: 'document.pdf', size: 2500 },
  { name: 'image.jpg', size: 1800 },
  { name: 'video.mp4', size: 5000 }
];

// File upload process (with progress indication)
function uploadFile(file: FileUpload) {
  const fileDiv = document.createElement('div');
  fileDiv.style.marginTop = '5px';
  fileDiv.style.padding = '5px';
  fileDiv.style.border = '1px solid #ccc';
  progressArea.appendChild(fileDiv);

  const progressSteps = [0, 25, 50, 75, 100];

  return from(progressSteps).pipe(
    delay(200),
    tap(progress => {
      fileDiv.textContent = `📄 ${file.name} (${file.size}KB) - ${progress}%`;
      if (progress === 100) {
        fileDiv.style.backgroundColor = '#e8f5e9';
      }
    }),
    ignoreElements() // Ignore progress values, notify only completion
  );
}

button.addEventListener('click', () => {
  button.disabled = true;
  progressArea.innerHTML = '';
  result.style.display = 'none';

  // Upload all files sequentially
  from(files).pipe(
    mergeMap(file => uploadFile(file), 2) // Upload up to23 files in parallel
  ).subscribe({
    complete: () => {
      result.style.display = 'block';
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
      result.innerHTML = `
        <strong>✅ Upload Complete</strong><br>
        ${files.length}One file uploaded
      `;
      button.disabled = false;
    },
    error: err => {
      result.style.display = 'block';
      result.style.backgroundColor = '#ffebee';
      result.style.color = 'red';
      result.textContent = `❌ Error: ${err.message}`;
      button.disabled = false;
    }
  });
});
```

- The progress of each file is displayed, but the progress values themselves do not flow downstream.
- A completion message is displayed when all uploads are complete.

## 🆚 Comparison with similar operators

### ignoreElements vs filter(() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs';

const source$ = of(1, 2, 3);

// ignoreElements: Ignore all values, completion is passed through
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('ignoreElements: Completed')
});
// Output: ignoreElements: Completed

// filter(() => false): Filter all values, let completion through
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('filter: Completed')
});
// Output: filter: Completed

// take(0): Completed immediately
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('take(0): Completed')
});
// Output: take(0): Completed
```

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Value:', value), // Not called
  complete: () => console.log('Completed')
});
// Output: Completed
```

**Recommended**: Use `ignoreElements()` if you intentionally want to ignore all values. The intent of the code will be clear.

## 🔄 Handling of error notifications

`ignoreElements` ignores the values, but **passes through error notifications**.


```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Error occurs'))
).pipe(
  ignoreElements()
);

// Success case
success$.subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('✅ Completed'),
  error: err => console.error('❌ Error:', err.message)
});
// Output: ✅ Completed

// Error case
error$.subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('✅ Completed'),
  error: err => console.error('❌ Error:', err.message)
});
// Output: ❌ Error: Error occurs
```

## ⚠️ Notes.

### 1. side effects will run

Although `ignoreElements` ignores values, side-effects (such as `tap`) will be executed.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Side Effects:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Value:', v),
  complete: () => console.log('Completed')
});
// Output:
// Side Effects: 1
// Side Effects: 2
// Side Effects: 3
// Completed
```

### 2. use with InfiniteObservable

When used with Infinite Observable, the subscription continues forever because completion never comes.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Bad case: Not completed
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Completed') // Not called
});

// ✅ Good example: take Completed in
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Completed') // 5Called after 2 seconds
});
```

### 3. Types in TypeScript

The return value of `ignoreElements` is of type `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// ignoreElements The result of Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value is of type never type, so this block is not executed
    console.log(value);
  },
  complete: () => console.log('Completion only')
});
```

### 4. when completion is not guaranteed

If the source does not complete, `ignoreElements` will not complete either.

```ts
import { NEVER } from 'rxjs';
import { ignoreElements } from 'rxjs';

// ❌ NEVERwill neither complete nor issue an error
NEVER.pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Completed') // Not called
});
```

## 💡 Practical Combination Patterns

### Pattern 1: Initialization sequence

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Value:', value), // Not called
  complete: () => console.log('Completed')
});
// Output: Completed
```

### Pattern 2: Cleanup process


```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Value:', value), // Not called
  complete: () => console.log('Completed')
});
// Output: Completed
```

## 📚 Related Operators

- **[filter](. /filter)** - filter values based on conditions
- **[take](. /take)** - take only the first N values
- **[skip](. /skip)** - skip the first N values
- **[tap](. /utility/tap)** - perform side action

## Summary

The `ignoreElements` operator ignores all values and only passes through completions and errors.

- ✅ Ideal when only notification of completion is needed
- ✅ Side effects (TAP) are executed
- ✅ Error notifications are also passed through
- ✅ Clearer intent than `filter(() => false)`.
- ⚠️ Infinite Observable does not complete
- ⚠️ Return value type is `Observable<never>`.
- ⚠️ Value is completely ignored, but side effects are performed
