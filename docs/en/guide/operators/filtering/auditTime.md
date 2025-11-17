---
description: auditTime is an RxJS filtering operator that waits for a specified time after a value is emitted and outputs the last value within that period. It is ideal when you want to periodically sample the latest state of high frequency events such as scroll position tracking, window resizing, and mouse movement. It is important to understand the difference from throttleTime and debounceTime and use them appropriately.
titleTemplate: ':title | RxJS'
---

# auditTime - Emit Last Value After Specified Time

The `auditTime` operator waits for a **specified time** after a value is emitted and outputs the **last value** within that time period. It then waits for the next value.


## 🔰 Basic Syntax and Usage

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Click!'));
```

**Flow of operation**:
1. First click occurs
2. Wait 1 second (clicks during this time are recorded but not output)
3. Output the last click after 1 second
4. Wait for the next click

[🌐 RxJS Official Documentation - `auditTime`](https://rxjs.dev/api/operators/auditTime)


## 🆚 Contrast with throttleTime

`throttleTime` and `auditTime` are similar, but output different values.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Output the first value
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Output: 0, 4, 8 (first value of each period)

// auditTime: Output the last value
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Output: 3, 6, 9 (last value of each period)
```

**Timeline comparison**:
```
Source:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (first)  (first)  (first)

audit:      -------3--------6--------9----|
                  (last)   (last)   (last)
```

| Operator | Output Value | Output Timing | Use Case |
|---|---|---|---|
| `throttleTime(ms)` | **First** value within period | Upon value reception | Immediate reaction needed |
| `auditTime(ms)` | **Last** value within period | At period end | Latest state needed |
| `debounceTime(ms)` | **Last** value after silence | After input stops | Wait for input completion |


## 💡 Typical Usage Patterns

1. **Window Resize Optimization**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // Get latest size every 200ms
   ).subscribe(() => {
     console.log(`Window size: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Scroll Position Tracking**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map } from 'rxjs';

   fromEvent(window, 'scroll').pipe(
     auditTime(100),
     map(() => ({
       scrollY: window.scrollY,
       scrollX: window.scrollX
     }))
   ).subscribe(position => {
     console.log(`Scroll position: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```


## 🎯 Difference from debounceTime

`auditTime` and `debounceTime` both **output the last value**, but **timing is completely different**.

### Key Difference

| Operator | Behavior | Use Case |
|---|---|---|
| `auditTime(ms)` | **Always outputs after ms** once value arrives (even if input continues) | Want to sample periodically |
| `debounceTime(ms)` | Outputs after ms **after input stops** | Want to wait for input completion |

### Concrete Example: Difference in Search Input

```ts
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Enter search keywords';
document.body.appendChild(input);

// auditTime: Execute search every 300ms even while typing
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Search:', input.value);
});

// debounceTime: Execute search 300ms after typing stops
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Search:', input.value);
});
```

### Timeline Visualization

When user types "ab" → "abc" → "abcd" quickly:

```
Input events:   a--b--c--d------------|
              ↓
auditTime:    ------c-----d----------|
            (after 300ms) (after 300ms)
            → Search "abc", search "abcd" (2 times total)

debounceTime: --------------------d-|
                              (300ms after stop)
            → Search "abcd" (1 time only)
```

**Easy Reminder**:
- **`auditTime`**: "Periodically audit" → Check at regular intervals
- **`debounceTime`**: "Wait until settled (debounce)" → Wait until quiet


## 🧠 Practical Code Example (Mouse Tracking)

Example of tracking mouse movement and displaying the latest position at regular intervals.

```ts
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// Create UI elements
const container = document.createElement('div');
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'Move your mouse within this area';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
positionDisplay.style.fontFamily = 'monospace';
document.body.appendChild(positionDisplay);

const dot = document.createElement('div');
dot.style.width = '10px';
dot.style.height = '10px';
dot.style.borderRadius = '50%';
dot.style.backgroundColor = '#e74c3c';
dot.style.position = 'absolute';
dot.style.display = 'none';
container.appendChild(dot);

// Mouse move event
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Get latest position every 100ms
).subscribe(position => {
  positionDisplay.textContent = `Latest position (100ms interval): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Move dot to latest position
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});
```

This code gets and displays only the latest position every 100ms, even when the mouse is moving frequently.


## 🎓 Summary

### When to Use auditTime
- ✅ When you need the latest value at regular intervals
- ✅ High frequency events like scroll, resize, mouse movement
- ✅ When periodic sampling is needed
- ✅ When you want to reflect the latest state

### When to Use throttleTime
- ✅ When immediate reaction is needed
- ✅ When you want to start processing with the first value
- ✅ Prevent button mashing

### When to Use debounceTime
- ✅ When you want to wait for input completion
- ✅ Search, autocomplete
- ✅ Wait until user stops typing

### Notes
- ⚠️ `auditTime` outputs only the last value within the period (intermediate values are discarded)
- ⚠️ If set to a short interval, it may not be very effective
- ⚠️ Depending on the use case, `throttleTime` or `debounceTime` may be more appropriate


## 🚀 Next Steps

- **[throttleTime](/en/guide/operators/filtering/throttleTime)** - Learn how to pass through the first value
- **[debounceTime](/en/guide/operators/filtering/debounceTime)** - Learn how to emit values after input stops
- **[filter](/en/guide/operators/filtering/filter)** - Learn how to filter based on conditions
- **[Filtering Operator Practical Examples](/en/guide/operators/filtering/practical-use-cases)** - Learn real use cases
